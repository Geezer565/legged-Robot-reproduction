#!/usr/bin/env python3

import argparse
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shlex
import shutil
import signal
import statistics
import subprocess

import myrbf_v5a_campaign_schema as campaign_schema


SOURCE_ROOT = Path(__file__).resolve().parents[2]
COMMAND_TOLERANCE = 1e-9


@dataclass(frozen=True)
class RunSpec:
    method: str
    scenario: str
    repeat: int
    seed: int
    order: int
    config_path: str


def build_run_matrix() -> list:
    return [
        RunSpec(
            method=method,
            scenario=scenario,
            repeat=repeat,
            seed=seed,
            order=order,
            config_path=campaign_schema.METHOD_CONFIGS[method],
        )
        for method, scenario, repeat, seed, order
        in campaign_schema.expected_run_identities()
    ]


def command_for(spec, output_dir: Path) -> list:
    output_dir = Path(output_dir)
    outputs = {
        "state_output": output_dir / "state.csv",
        "shadow_output": output_dir / "shadow.csv",
        "wrench_output": output_dir / "wrench.csv",
        "motion_output": output_dir / "motion.csv",
    }
    return list(campaign_schema.LAUNCH_PREFIX) + [
        "%s:=%s" % (name, value)
        for name, value in campaign_schema.ordered_launch_arguments(
            spec.method, spec.scenario, spec.seed, outputs)
    ]


def run_id_for(spec):
    method_slug = spec.method.lower().replace("-", "")
    return "%s_r%02d_%02d_%s" % (
        spec.scenario, spec.repeat, spec.order, method_slug)


def _utc_now():
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _source_head():
    return subprocess.check_output(
        ["git", "rev-parse", "HEAD"], cwd=str(SOURCE_ROOT), text=True
    ).strip()


def _validate_source_commit(source_commit, source_head):
    if (not isinstance(source_commit, str) or len(source_commit) != 40 or
            any(character not in "0123456789abcdef"
                for character in source_commit)):
        raise ValueError("source commit must be an explicit lowercase 40-hex value")
    if source_commit != source_head:
        raise ValueError("source commit does not match source HEAD")


def _validate_frozen_inputs(protocol, source_commit, source_head):
    protocol = Path(protocol) if protocol is not None else None
    if protocol is None or not protocol.is_file():
        raise ValueError("execution requires an explicit protocol file")
    protocol_hash = _sha256(protocol)
    if protocol_hash != campaign_schema.FROZEN_PROTOCOL_SHA256:
        raise ValueError("protocol hash does not match frozen campaign")
    _validate_source_commit(source_commit, source_head)
    for method in campaign_schema.METHODS:
        config_path = SOURCE_ROOT / campaign_schema.METHOD_CONFIGS[method]
        if _sha256(config_path) != campaign_schema.METHOD_CONFIG_SHA256[method]:
            raise ValueError("configuration hash drift for %s" % method)
    return protocol


def _read_csv(path, required_fields, string_fields=(),
              skip_leading_nonfinite=False):
    path = Path(path)
    if not path.is_file():
        raise ValueError("required CSV is missing: %s" % path.name)
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        fields = tuple(reader.fieldnames or ())
        missing = [field for field in required_fields if field not in fields]
        if missing:
            raise ValueError("%s missing required fields: %s" %
                             (path.name, ", ".join(missing)))
        rows = []
        usable_started = False
        for row_number, source in enumerate(reader, start=2):
            row = dict(source)
            finite = True
            for field in required_fields:
                if field in string_fields:
                    if row.get(field) is None:
                        finite = False
                    continue
                try:
                    value = float(row[field])
                except (KeyError, TypeError, ValueError):
                    finite = False
                    break
                if not math.isfinite(value):
                    finite = False
                    break
                row[field] = value
            if not finite:
                if skip_leading_nonfinite and not usable_started:
                    continue
                raise ValueError("%s row %d has a nonfinite usable value" %
                                 (path.name, row_number))
            usable_started = True
            rows.append(row)
    if not rows:
        raise ValueError("%s has no usable samples" % path.name)
    return rows


def _validate_series(rows, field, label):
    if len(rows) < 3:
        raise ValueError("%s has too few usable samples" % label)
    differences = [right[field] - left[field]
                   for left, right in zip(rows, rows[1:])]
    if any(value <= 0.0 for value in differences):
        raise ValueError("%s must be strictly increasing" % label)
    period = float(statistics.median(differences))
    if (max(differences) >
            campaign_schema.MAX_SAMPLE_GAP_FACTOR * period + 1e-12):
        raise ValueError("%s has an excessive gap" % label)


def _span_claim(rows, field):
    start = rows[0][field]
    stop = rows[-1][field]
    return {
        "rows": len(rows),
        "start_s": start,
        "stop_s": stop,
        "span_s": stop - start,
    }


def _validate_full_run(claim, start_elapsed, duration, label):
    allowed_start = (
        duration *
        campaign_schema.FULL_RUN_COVERAGE["start_margin_fraction"]
    )
    required_span = (
        duration *
        campaign_schema.FULL_RUN_COVERAGE["minimum_span_fraction"]
    )
    if abs(start_elapsed) > allowed_start + 1e-12:
        raise ValueError("%s full-run coverage start margin is exceeded" % label)
    if claim["span_s"] + 1e-12 < required_span:
        raise ValueError("%s full-run coverage is below the shared minimum" %
                         label)


def _validate_motion(rows, scenario):
    start = scenario["motion_delay_s"]
    stop = start + scenario["motion_duration_s"]
    if rows[0]["wall_elapsed"] > start or rows[-1]["wall_elapsed"] < stop:
        raise ValueError("motion coverage does not cover the exact schedule")
    for row in rows:
        active = start <= row["wall_elapsed"] < stop
        expected = (
            scenario["vx_mps"] if active else 0.0,
            scenario["vy_mps"] if active else 0.0,
            scenario["yaw_rate_rps"] if active else 0.0,
        )
        if ((row["active"] > 0.5) != active or
                row["gait"] != (scenario["gait"] if active else "stance") or
                any(abs(value - wanted) > COMMAND_TOLERANCE
                    for value, wanted in zip(
                        (row["vx"], row["vy"], row["yaw_rate"]), expected))):
            raise ValueError("motion record does not match the exact schedule")


def _expected_force(scenario, elapsed):
    start = scenario["force_onset_s"]
    if start is None or not start <= elapsed < start + scenario["force_duration_s"]:
        return 0.0
    if scenario["force_profile"] == "constant":
        return scenario["force_value_n"]
    return (scenario["force_value_n"] +
            scenario["force_amplitude_n"] * math.sin(
                2.0 * math.pi * scenario["force_frequency_hz"] *
                (elapsed - start)))


def _validate_wrench(rows, scenario):
    if not scenario["wrench_required"]:
        if any(abs(row[field]) > COMMAND_TOLERANCE
               for row in rows
               for field in ("fx", "fy", "fz", "tx", "ty", "tz")):
            raise ValueError("no-disturbance wrench must be all zero")
        return
    start = scenario["force_onset_s"]
    stop = start + scenario["force_duration_s"]
    if rows[0]["wall_elapsed"] > start or rows[-1]["wall_elapsed"] < stop:
        raise ValueError("wrench coverage does not cover the exact schedule")
    for row in rows:
        active = start <= row["wall_elapsed"] < stop
        if ((row["service_called"] > 0.5) != active or
                row["service_success"] < 0.5 or
                abs(row[scenario["force_axis"]] -
                    _expected_force(scenario, row["wall_elapsed"])) > 1e-6):
            raise ValueError("wrench record does not match the exact schedule")


def _safety_stop_detected(log_path):
    text = Path(log_path).read_text(encoding="utf-8", errors="replace").lower()
    return any(marker.lower() in text
               for marker in campaign_schema.SAFETY_STOP_MARKERS)


def _inspect_attempt(spec, attempt_dir, log_path):
    scenario = campaign_schema.SCENARIOS[spec.scenario]
    duration = scenario["duration_s"]
    shadow = _read_csv(
        attempt_dir / "shadow.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["shadow.csv"],
    )
    state = _read_csv(
        attempt_dir / "state.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["state.csv"],
        skip_leading_nonfinite=True,
    )
    motion = _read_csv(
        attempt_dir / "motion.csv",
        campaign_schema.REQUIRED_CSV_FIELDS["motion.csv"],
        string_fields=("gait",),
    )
    wrench_path = attempt_dir / "wrench.csv"
    wrench = None
    if scenario["wrench_required"] or wrench_path.is_file():
        wrench = _read_csv(
            wrench_path,
            campaign_schema.REQUIRED_CSV_FIELDS["wrench.csv"],
        )

    _validate_series(shadow, "time_s", "controller times")
    _validate_series(state, "ros_time", "state times")
    _validate_series(motion, "wall_elapsed", "motion times")
    if wrench is not None:
        _validate_series(wrench, "wall_elapsed", "wrench times")
    controller_claim = _span_claim(shadow, "time_s")
    estimator_claim = _span_claim(state, "wall_elapsed")
    ros_origin = state[0]["ros_time"] - state[0]["wall_elapsed"]
    _validate_full_run(
        controller_claim, shadow[0]["time_s"] - ros_origin,
        duration, "controller")
    _validate_full_run(
        estimator_claim, state[0]["wall_elapsed"], duration, "state")
    _validate_motion(motion, scenario)
    if wrench is not None:
        _validate_wrench(wrench, scenario)
    minimum_height = min(row["base_z"] for row in state)
    safety_stop = _safety_stop_detected(log_path)
    if minimum_height < campaign_schema.GATE_THRESHOLDS["safety"][
            "minimum_base_height_m"]:
        raise ValueError("minimum base height is below the safety gate")
    if safety_stop:
        raise ValueError("recognized safety-stop marker found in run.log")
    return {
        "controller_rows": len(shadow),
        "estimator_rows": len(state),
        "observed_spans": {
            "controller": controller_claim,
            "estimator": estimator_claim,
            "motion": _span_claim(motion, "wall_elapsed"),
            "wrench": (_span_claim(wrench, "wall_elapsed")
                       if wrench is not None else None),
        },
        "minimum_base_height_m": minimum_height,
        "safety_stop_detected": safety_stop,
        "finite": True,
    }


def _supervise(command, cwd, log_path, timeout_s):
    with Path(log_path).open("a", encoding="utf-8") as log_handle:
        process = subprocess.Popen(
            list(command), cwd=str(cwd), stdout=log_handle,
            stderr=subprocess.STDOUT, start_new_session=True,
        )
        try:
            return {"exit_code": process.wait(timeout=timeout_s),
                    "timed_out": False}
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
            return {"exit_code": process.returncode, "timed_out": True}


def _write_manifest(path, manifest):
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _base_run_record(spec, source_commit, command):
    return {
        "run_id": run_id_for(spec),
        "method": spec.method,
        "scenario": spec.scenario,
        "repeat": spec.repeat,
        "seed": spec.seed,
        "order": spec.order,
        "paired_conditions": campaign_schema.paired_conditions(
            spec.scenario, spec.seed),
        "git_commit": source_commit,
        "config_path": spec.config_path,
        "config_sha256": campaign_schema.METHOD_CONFIG_SHA256[spec.method],
        "command": list(command),
    }


def _safe_output_directory(output_root, relative_output):
    relative = Path(str(relative_output))
    if relative.is_absolute():
        raise ValueError("resume output directory drift: absolute path")
    resolved_root = output_root.resolve()
    resolved = (resolved_root / relative).resolve()
    try:
        resolved.relative_to(resolved_root)
    except ValueError:
        raise ValueError("resume output directory drift: path escapes root")
    return resolved


def _inspection_matches_run(inspection, run):
    return (
        run.get("controller_rows") == inspection["controller_rows"] and
        run.get("estimator_rows") == inspection["estimator_rows"] and
        run.get("observed_spans") == inspection["observed_spans"] and
        run.get("finite") is True and
        run.get("safety_stop_detected") is False and
        run.get("exit_code") == 0
    )


def _load_resume_manifest(output_root, matrix, source_commit):
    manifest_path = output_root / "manifest.json"
    if not manifest_path.is_file():
        raise ValueError("resume manifest is missing")
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ValueError("resume manifest is invalid: %s" % error)
    if (not isinstance(manifest, dict) or
            manifest.get("schema_version") != campaign_schema.SCHEMA_VERSION or
            manifest.get("protocol_sha256") !=
            campaign_schema.FROZEN_PROTOCOL_SHA256 or
            manifest.get("protocol_snapshot") !=
            campaign_schema.protocol_snapshot()):
        raise ValueError("resume protocol/schema drift")
    frozen_protocol = output_root / campaign_schema.PROTOCOL_FILENAME
    if (not frozen_protocol.is_file() or
            _sha256(frozen_protocol) != campaign_schema.FROZEN_PROTOCOL_SHA256):
        raise ValueError("resume protocol file drift")
    runs = manifest.get("runs")
    if not isinstance(runs, list) or len(runs) > len(matrix):
        raise ValueError("resume run list drift")
    required = set(campaign_schema.REQUIRED_RUN_FIELDS)
    for index, run in enumerate(runs):
        if not isinstance(run, dict) or not required.issubset(run):
            raise ValueError("resume manifest run drift")
        spec = matrix[index]
        identity = (run.get("method"), run.get("scenario"),
                    run.get("repeat"), run.get("seed"), run.get("order"))
        expected_identity = (spec.method, spec.scenario, spec.repeat,
                             spec.seed, spec.order)
        if run.get("run_id") != run_id_for(spec) or identity != expected_identity:
            raise ValueError("resume run identity drift")
        if run.get("git_commit") != source_commit:
            raise ValueError("resume source commit drift")
        if (run.get("config_path") != spec.config_path or
                run.get("config_sha256") !=
                campaign_schema.METHOD_CONFIG_SHA256[spec.method]):
            raise ValueError("resume configuration drift")
        if run.get("paired_conditions") != campaign_schema.paired_conditions(
                spec.scenario, spec.seed):
            raise ValueError("resume paired-condition drift")
        history = run.get("attempt_history")
        if not isinstance(history, list) or not history:
            raise ValueError("resume attempt history drift")
        for attempt_number, attempt in enumerate(history, start=1):
            if (not isinstance(attempt, dict) or
                    attempt.get("attempt") != attempt_number):
                raise ValueError("resume attempt history drift")
            attempt_dir = _safe_output_directory(
                output_root, attempt.get("output_dir", ""))
            if not attempt_dir.is_dir():
                raise ValueError("resume attempt directory drift")
        latest = history[-1]
        if (run.get("output_dir") != latest.get("output_dir") or
                run.get("command") != latest.get("command")):
            raise ValueError("resume latest-attempt drift")
        latest_dir = _safe_output_directory(output_root, run["output_dir"])
        expected_command = command_for(spec, latest_dir)
        if run["command"] != expected_command:
            raise ValueError("resume command drift")
        if run.get("complete"):
            log_path = latest_dir / "run.log"
            if not log_path.is_file():
                raise ValueError("resume complete evidence drift")
            try:
                inspection = _inspect_attempt(spec, latest_dir, log_path)
            except ValueError as error:
                raise ValueError("resume complete evidence drift: %s" % error)
            if not _inspection_matches_run(inspection, run):
                raise ValueError("resume complete row/span drift")
    return manifest


def run_campaign(output_root: Path, execute: bool, **options) -> int:
    output_root = Path(output_root)
    matrix = build_run_matrix()
    if not execute:
        for spec in matrix:
            attempt_dir = output_root / run_id_for(spec) / "attempt-001"
            print(shlex.join(command_for(spec, attempt_dir)))
        return 0

    resume = bool(options.get("resume", False))
    if not resume and output_root.exists() and any(output_root.iterdir()):
        raise ValueError("first execution refuses a nonempty output root")
    source_head = options.get("source_head") or _source_head()
    source_commit = options.get("source_commit")
    protocol = _validate_frozen_inputs(
        options.get("protocol"), source_commit, source_head)
    manifest_path = output_root / "manifest.json"
    if resume:
        if not output_root.is_dir():
            raise ValueError("resume campaign root is missing")
        manifest = _load_resume_manifest(output_root, matrix, source_commit)
    else:
        output_root.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(
            str(protocol),
            str(output_root / campaign_schema.PROTOCOL_FILENAME),
        )
        manifest = {
            "schema_version": campaign_schema.SCHEMA_VERSION,
            "protocol_sha256": campaign_schema.FROZEN_PROTOCOL_SHA256,
            "protocol_snapshot": campaign_schema.protocol_snapshot(),
            "runs": [],
        }
        _write_manifest(manifest_path, manifest)
    process_runner = options.get("process_runner") or _supervise
    timeout_padding = float(options.get("timeout_padding_s", 30.0))
    for matrix_index, spec in enumerate(matrix):
        run_id = run_id_for(spec)
        run_dir = output_root / run_id
        existing = (manifest["runs"][matrix_index]
                    if matrix_index < len(manifest["runs"]) else None)
        if existing is not None and existing.get("complete"):
            continue
        history = (list(existing["attempt_history"])
                   if existing is not None else [])
        attempt_number = len(history) + 1
        if existing is None:
            run_dir.mkdir(exist_ok=False)
        elif not run_dir.is_dir():
            raise ValueError("resume run directory drift")
        attempt_dir = run_dir / ("attempt-%03d" % attempt_number)
        attempt_dir.mkdir(exist_ok=False)
        relative_output = attempt_dir.relative_to(output_root).as_posix()
        command = command_for(spec, attempt_dir)
        log_path = attempt_dir / "run.log"
        log_path.touch(exist_ok=False)
        started_at = _utc_now()
        process_result = {"exit_code": None, "timed_out": False}
        inspection = {
            "controller_rows": 0,
            "estimator_rows": 0,
            "observed_spans": {stream: None
                               for stream in campaign_schema.OBSERVED_STREAMS},
            "minimum_base_height_m": None,
            "safety_stop_detected": False,
            "finite": False,
        }
        error = None
        try:
            process_result = process_runner(
                command, SOURCE_ROOT, log_path,
                campaign_schema.SCENARIOS[spec.scenario]["duration_s"] +
                timeout_padding,
            )
            inspection["safety_stop_detected"] = _safety_stop_detected(log_path)
            if (process_result.get("exit_code") == 0 and
                    not process_result.get("timed_out")):
                inspection = _inspect_attempt(spec, attempt_dir, log_path)
        except (OSError, ValueError) as caught:
            error = str(caught)
            inspection["safety_stop_detected"] = _safety_stop_detected(log_path)
        finished_at = _utc_now()
        complete = (
            process_result.get("exit_code") == 0 and
            not process_result.get("timed_out") and
            inspection["finite"] and
            not inspection["safety_stop_detected"] and
            error is None
        )
        attempt = {
            "attempt": attempt_number,
            "output_dir": relative_output,
            "command": list(command),
            "started_at": started_at,
            "finished_at": finished_at,
            "exit_code": process_result.get("exit_code"),
            "timed_out": bool(process_result.get("timed_out")),
            "controller_rows": inspection["controller_rows"],
            "estimator_rows": inspection["estimator_rows"],
            "observed_spans": inspection["observed_spans"],
            "minimum_base_height_m": inspection["minimum_base_height_m"],
            "safety_stop_detected": inspection["safety_stop_detected"],
            "finite": inspection["finite"],
            "complete": complete,
            "error": error,
        }
        run = _base_run_record(spec, source_commit, command)
        history.append(attempt)
        run.update({
            "started_at": started_at,
            "finished_at": finished_at,
            "exit_code": process_result.get("exit_code"),
            "controller_rows": inspection["controller_rows"],
            "estimator_rows": inspection["estimator_rows"],
            "finite": inspection["finite"],
            "safety_stop_detected": inspection["safety_stop_detected"],
            "complete": complete,
            "output_dir": relative_output,
            "attempt_history": history,
            "observed_spans": inspection["observed_spans"],
            "minimum_base_height_m": inspection["minimum_base_height_m"],
        })
        if existing is None:
            manifest["runs"].append(run)
        else:
            manifest["runs"][matrix_index] = run
        _write_manifest(manifest_path, manifest)
        if not complete:
            return 1
    return 0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run the frozen MyRBF V4/V5-A paired campaign.")
    parser.add_argument(
        "--output-root", type=Path,
        default=Path("/tmp/myrbf_v5a_repeat_20260901"),
    )
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--dry-run", action="store_true")
    mode.add_argument("--execute", action="store_true")
    parser.add_argument("--protocol", type=Path)
    parser.add_argument("--source-commit")
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args(argv)
    if args.execute and args.protocol is None:
        parser.error("--execute requires explicit --protocol")
    if args.execute and args.source_commit is None:
        parser.error("--execute requires explicit --source-commit")
    if args.dry_run and args.resume:
        parser.error("--resume is valid only with --execute")
    return args


def main(argv=None):
    args = parse_args(argv)
    return run_campaign(
        args.output_root,
        execute=args.execute,
        protocol=args.protocol,
        source_commit=args.source_commit,
        resume=args.resume,
    )


if __name__ == "__main__":
    raise SystemExit(main())
