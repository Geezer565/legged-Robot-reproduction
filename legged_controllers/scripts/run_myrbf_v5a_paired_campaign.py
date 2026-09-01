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
import re
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


def _validate_source_commit(source_commit):
    if (not isinstance(source_commit, str) or len(source_commit) != 40 or
            any(character not in "0123456789abcdef"
                for character in source_commit)):
        raise ValueError("source commit must be an explicit lowercase 40-hex value")


def _canonical_hash(value):
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def create_source_lock(source_root, source_commit):
    source_root = Path(source_root)
    _validate_source_commit(source_commit)
    try:
        source_head = subprocess.check_output(
            ["git", "rev-parse", "HEAD"], cwd=str(source_root), text=True,
            stderr=subprocess.STDOUT,
        ).strip()
    except (OSError, subprocess.CalledProcessError) as error:
        raise ValueError("Git is not usable for source-lock creation") from error
    if source_head != source_commit:
        raise ValueError("source commit does not match source HEAD")
    critical_files = tuple(campaign_schema.EXECUTION_CRITICAL_FILES)
    tracked = subprocess.check_output(
        ["git", "ls-files", "--", *critical_files],
        cwd=str(source_root), text=True,
    ).splitlines()
    if set(tracked) != set(critical_files):
        raise ValueError("source-lock critical files must all be tracked")
    dirty = subprocess.check_output(
        ["git", "status", "--porcelain", "--untracked-files=no", "--",
         *critical_files],
        cwd=str(source_root), text=True,
    ).strip()
    if dirty:
        raise ValueError("dirty critical tracked files prevent source-lock creation")
    return {
        "schema_version": campaign_schema.SOURCE_LOCK_VERSION,
        "source_commit": source_commit,
        "files": {
            relative: _sha256(source_root / relative)
            for relative in critical_files
        },
    }


def _verify_source_lock(source_root, source_lock):
    expected = tuple(campaign_schema.EXECUTION_CRITICAL_FILES)
    files = source_lock.get("files") if isinstance(source_lock, dict) else None
    if (not isinstance(source_lock, dict) or
            source_lock.get("schema_version") !=
            campaign_schema.SOURCE_LOCK_VERSION or
            not isinstance(files, dict) or
            set(files) != set(expected)):
        raise ValueError("source-lock schema/file set drift")
    for relative in expected:
        path = Path(source_root) / relative
        expected_hash = files.get(relative)
        if (not isinstance(expected_hash, str) or len(expected_hash) != 64 or
                not path.is_file() or _sha256(path) != expected_hash):
            raise ValueError("source-lock drift for %s" % relative)


def _load_source_lock(path, source_root, source_commit):
    if path is None or not Path(path).is_file():
        raise ValueError("execution requires an explicit source lock")
    try:
        source_lock = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise ValueError("source lock is invalid") from error
    _validate_source_commit(source_commit)
    if source_lock.get("source_commit") != source_commit:
        raise ValueError("source-lock commit drift")
    _verify_source_lock(source_root, source_lock)
    return source_lock


def _validate_frozen_inputs(protocol, source_commit, source_lock_path,
                            source_root):
    protocol = Path(protocol) if protocol is not None else None
    if protocol is None or not protocol.is_file():
        raise ValueError("execution requires an explicit protocol file")
    protocol_hash = _sha256(protocol)
    if protocol_hash != campaign_schema.FROZEN_PROTOCOL_SHA256:
        raise ValueError("protocol hash does not match frozen campaign")
    source_lock = _load_source_lock(
        source_lock_path, source_root, source_commit)
    return protocol, source_lock, _canonical_hash(source_lock)


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


def _validate_shared_campaign_start(streams):
    first_stream = next(iter(streams.values()))
    campaign_start_ros = first_stream[0]["campaign_start_ros"]
    for label, rows in streams.items():
        if any(abs(row["campaign_start_ros"] - campaign_start_ros) > 1e-9
               for row in rows):
            raise ValueError("%s campaign start does not match shared origin" %
                             label)
    return campaign_start_ros


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
    selected_axis = scenario["force_axis"]
    for row in rows:
        if any(abs(row[axis]) > COMMAND_TOLERANCE
               for axis in ("fx", "fy", "fz", "tx", "ty", "tz")
               if axis != selected_axis):
            raise ValueError("non-selected wrench axis must remain zero")
        active = start <= row["wall_elapsed"] < stop
        if ((row["service_called"] > 0.5) != active or
                row["service_success"] < 0.5 or
                abs(row[selected_axis] -
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

    streams = {"shadow": shadow, "state": state, "motion": motion}
    if wrench is not None:
        streams["wrench"] = wrench
    campaign_start_ros = _validate_shared_campaign_start(streams)

    _validate_series(shadow, "time_s", "controller times")
    _validate_series(state, "ros_time", "state times")
    _validate_series(motion, "wall_elapsed", "motion times")
    if wrench is not None:
        _validate_series(wrench, "wall_elapsed", "wrench times")
    controller_claim = _span_claim(shadow, "time_s")
    estimator_claim = _span_claim(state, "wall_elapsed")
    _validate_full_run(
        controller_claim, shadow[0]["time_s"] - campaign_start_ros,
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
        "campaign_start_ros": campaign_start_ros,
    }


def _terminate_process_group(process):
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


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
            _terminate_process_group(process)
            return {"exit_code": process.returncode, "timed_out": True}
        except BaseException:
            _terminate_process_group(process)
            raise


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
        run.get("campaign_start_ros") == inspection["campaign_start_ros"] and
        run.get("finite") is True and
        run.get("safety_stop_detected") is False and
        run.get("exit_code") == 0
    )


def _load_resume_manifest(output_root, matrix, source_commit, source_lock,
                          source_lock_sha256):
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
            campaign_schema.protocol_snapshot() or
            manifest.get("source_lock") != source_lock or
            manifest.get("source_lock_sha256") != source_lock_sha256):
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
        previous_attempt = 0
        for attempt in history:
            attempt_number = (
                attempt.get("attempt") if isinstance(attempt, dict) else None
            )
            if (not isinstance(attempt_number, int) or
                    attempt_number <= previous_attempt):
                raise ValueError("resume attempt history drift")
            previous_attempt = attempt_number
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


def _next_attempt_number(run_dir, history):
    used = {
        attempt["attempt"] for attempt in history
        if isinstance(attempt, dict) and isinstance(attempt.get("attempt"), int)
    }
    if run_dir.is_dir():
        for child in run_dir.iterdir():
            match = re.fullmatch(r"attempt-(\d{3})", child.name)
            if match:
                used.add(int(match.group(1)))
    return max(used, default=0) + 1


def run_campaign(output_root: Path, execute: bool, **options) -> int:
    output_root = Path(output_root).resolve()
    matrix = build_run_matrix()
    if not execute:
        for spec in matrix:
            attempt_dir = output_root / run_id_for(spec) / "attempt-001"
            print(shlex.join(command_for(spec, attempt_dir)))
        return 0

    resume = bool(options.get("resume", False))
    if not resume and output_root.exists() and any(output_root.iterdir()):
        raise ValueError("first execution refuses a nonempty output root")
    source_commit = options.get("source_commit")
    source_root = Path(options.get("source_root", SOURCE_ROOT))
    protocol, source_lock, source_lock_sha256 = _validate_frozen_inputs(
        options.get("protocol"), source_commit, options.get("source_lock"),
        source_root)
    manifest_path = output_root / "manifest.json"
    if resume:
        if not output_root.is_dir():
            raise ValueError("resume campaign root is missing")
        manifest = _load_resume_manifest(
            output_root, matrix, source_commit, source_lock,
            source_lock_sha256)
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
            "source_lock": source_lock,
            "source_lock_sha256": source_lock_sha256,
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
        if existing is None:
            if resume and run_dir.is_dir():
                pass
            else:
                run_dir.mkdir(exist_ok=False)
        elif not run_dir.is_dir():
            raise ValueError("resume run directory drift")
        attempt_number = _next_attempt_number(run_dir, history)
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
            "campaign_start_ros": None,
        }
        attempt = {
            "attempt": attempt_number,
            "status": "in_progress",
            "output_dir": relative_output,
            "command": list(command),
            "started_at": started_at,
            "finished_at": None,
            "exit_code": None,
            "timed_out": False,
            "controller_rows": 0,
            "estimator_rows": 0,
            "observed_spans": inspection["observed_spans"],
            "minimum_base_height_m": None,
            "safety_stop_detected": False,
            "finite": False,
            "campaign_start_ros": None,
            "complete": False,
            "error": None,
        }
        run = _base_run_record(spec, source_commit, command)
        history.append(attempt)
        run.update({
            "started_at": started_at,
            "finished_at": None,
            "exit_code": None,
            "controller_rows": 0,
            "estimator_rows": 0,
            "finite": False,
            "campaign_start_ros": None,
            "safety_stop_detected": False,
            "complete": False,
            "output_dir": relative_output,
            "attempt_history": history,
            "observed_spans": inspection["observed_spans"],
            "minimum_base_height_m": None,
        })
        if existing is None:
            manifest["runs"].append(run)
        else:
            manifest["runs"][matrix_index] = run
        _write_manifest(manifest_path, manifest)
        _verify_source_lock(source_root, source_lock)
        error = None
        try:
            process_result = process_runner(
                command, source_root, log_path,
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
        attempt.update({
            "attempt": attempt_number,
            "status": "complete" if complete else "failed",
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
            "campaign_start_ros": inspection["campaign_start_ros"],
            "complete": complete,
            "error": error,
        })
        run.update({
            "started_at": started_at,
            "finished_at": finished_at,
            "exit_code": process_result.get("exit_code"),
            "controller_rows": inspection["controller_rows"],
            "estimator_rows": inspection["estimator_rows"],
            "finite": inspection["finite"],
            "campaign_start_ros": inspection["campaign_start_ros"],
            "safety_stop_detected": inspection["safety_stop_detected"],
            "complete": complete,
            "output_dir": relative_output,
            "attempt_history": history,
            "observed_spans": inspection["observed_spans"],
            "minimum_base_height_m": inspection["minimum_base_height_m"],
        })
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
    parser.add_argument("--source-lock", type=Path)
    parser.add_argument("--resume", action="store_true")
    args = parser.parse_args(argv)
    if args.execute and args.protocol is None:
        parser.error("--execute requires explicit --protocol")
    if args.execute and args.source_commit is None:
        parser.error("--execute requires explicit --source-commit")
    if args.execute and args.source_lock is None:
        parser.error("--execute requires explicit --source-lock")
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
        source_lock=args.source_lock,
        resume=args.resume,
    )


if __name__ == "__main__":
    raise SystemExit(main())
