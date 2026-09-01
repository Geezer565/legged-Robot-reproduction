#!/usr/bin/env python3
"""Recompute the frozen V5-A repeat-campaign facts and validate its reports."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import sys
from pathlib import Path
from typing import Any


CHECKER_NAME = "myrbf-v5a-repeat-report-v3"
DERIVED_HOST_FILES = {"REPORT_CHECK.json", "RESULT_SUMMARY.md", "TEACHER_UPDATE.md"}
EXPECTED_IMMUTABLE_FILE_COUNT = 73
EXPECTED_IMMUTABLE_INVENTORY_SHA256 = "5991321634c68ea87f9bb912760917eaf7efaff1933d9bb9770970e060a11793"
EXPECTED_HASHES = {
    "manifest.json": "464cb053433891e84a6488d7869035e9bc36a994dc0e93c8c1dded6628a4eb85",
    "PROTOCOL.md": "e6004dbbbc69b81fc58a8567196d46adf84c14dd2eee1b1acee656cca924c484",
    "SOURCE_LOCK.json": "12ed2972641138ff41ef610ab10768e9bba12d3f1f8c398ec162ea6f8ace2778",
    "locked_inputs.sha256": "43a0d3315f69286977d51d779fd92cda277bf696691a4617b12aa3c0d92fca41",
    "preflight_failures/647c642/manifest.json": "664824a1d6495c2b798fa7675f6cfa47a9d6584c32d8c0cc33d3429dab4dd2d5",
    "preflight_failures/a13c11e/manifest.json": "8059856cd3ac85d107389cc4d8e9f600920bf14fea76a900bc8c5128d6cb1c9a",
}
EXPECTED_SOURCE_COMMIT = "f5dab9d71e20c786fc8e5866af47b9ed8829f116"
EXPECTED_SOURCE_LOCK_FILES = 2296
EXPECTED_PROTOCOL_SHA256 = "e6004dbbbc69b81fc58a8567196d46adf84c14dd2eee1b1acee656cca924c484"
EXPECTED_ATTEMPTS = {
    "attempt-001": {
        "minimum_base_z": 0.13523551383728658,
        "csv_file_row": 483,
        "campaign_elapsed_s": 9.836,
        "ros_time_s": 11.355,
        "maximum_state_gap_s": 0.229,
        "force_start_s": 6.016,
        "force_stop_s": 11.996,
        "qp_working_set_errors": 2807,
        "protocol_deviation": False,
    },
    "attempt-002": {
        "minimum_base_z": 0.13512079248997821,
        "csv_file_row": 495,
        "campaign_elapsed_s": 9.969,
        "ros_time_s": 11.490,
        "maximum_state_gap_s": 0.126,
        "force_start_s": 6.009,
        "force_stop_s": 11.989,
        "qp_working_set_errors": 2183,
        "protocol_deviation": True,
    },
    "attempt-003": {
        "minimum_base_z": 0.13499104233606257,
        "csv_file_row": 496,
        "campaign_elapsed_s": 9.884,
        "ros_time_s": 11.403,
        "maximum_state_gap_s": 0.022,
        "force_start_s": 6.002,
        "force_stop_s": 11.982,
        "qp_working_set_errors": 6468,
        "protocol_deviation": True,
    },
}


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def immutable_inventory(root: Path) -> tuple[dict[str, str], str]:
    mapping: dict[str, str] = {}
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        relative = path.relative_to(root).as_posix()
        if relative in DERIVED_HOST_FILES:
            continue
        mapping[relative] = sha256_file(path)
    canonical = json.dumps(mapping, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
    return mapping, hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def close(actual: float, expected: float, tolerance: float = 5e-10) -> bool:
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


def read_state_facts(path: Path) -> dict[str, Any]:
    rows: list[tuple[int, float, float, float]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"ros_time", "wall_elapsed", "base_z"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: missing state columns")
        for file_row, row in enumerate(reader, start=2):
            values = (file_row, float(row["ros_time"]), float(row["wall_elapsed"]), float(row["base_z"]))
            if not all(math.isfinite(value) for value in values[1:]):
                raise ValueError(f"{path}: nonfinite state value at row {file_row}")
            rows.append(values)
    if len(rows) < 2:
        raise ValueError(f"{path}: fewer than two state rows")
    minimum = min(rows, key=lambda item: item[3])
    maximum_gap = max(current[2] - previous[2] for previous, current in zip(rows, rows[1:]))
    return {
        "minimum_base_z": minimum[3],
        "csv_file_row": minimum[0],
        "campaign_elapsed_s": minimum[2],
        "ros_time_s": minimum[1],
        "maximum_state_gap_s": maximum_gap,
        "rows": len(rows),
    }


def read_wrench_facts(path: Path) -> dict[str, Any]:
    active: list[float] = []
    rows = 0
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"wall_elapsed", "fx", "fy", "fz", "tx", "ty", "tz"}
        if not required.issubset(reader.fieldnames or []):
            raise ValueError(f"{path}: missing wrench columns")
        for row in reader:
            rows += 1
            values = {name: float(row[name]) for name in required}
            if not all(math.isfinite(value) for value in values.values()):
                raise ValueError(f"{path}: nonfinite wrench value")
            if any(abs(values[name]) > 1e-12 for name in ("fx", "fz", "tx", "ty", "tz")):
                raise ValueError(f"{path}: a non-Fy wrench axis is nonzero")
            if abs(values["fy"] - 20.0) <= 1e-12:
                active.append(values["wall_elapsed"])
            elif abs(values["fy"]) > 1e-12:
                raise ValueError(f"{path}: unexpected Fy value {values['fy']}")
    if not active:
        raise ValueError(f"{path}: no 20 N Fy samples")
    return {"force_start_s": min(active), "force_stop_s": max(active), "rows": rows}


def require(errors: list[str], condition: bool, message: str) -> None:
    if not condition:
        errors.append(message)


def unsupported_comparison(text: str) -> bool:
    comparison = re.compile(
        r"(?:V5-A\s*(?:优于|好于|胜过|better\s+than|worse\s+than|劣于)\s*V4|"
        r"V4\s*(?:优于|好于|胜过|better\s+than|worse\s+than|劣于)\s*V5-A)",
        re.IGNORECASE,
    )
    negation = re.compile(r"不能|无法|不支持|没有证据|不可|cannot|no evidence|not support|invalid", re.IGNORECASE)
    return any(comparison.search(line) and not negation.search(line) for line in text.splitlines())


def require_chronology(errors: list[str], label: str, text: str, compact: bool = False) -> None:
    minima = ("0.1352355", "0.1351208", "0.1349910") if compact else (
        "0.13523551383728658",
        "0.13512079248997821",
        "0.13499104233606257",
    )
    for value in minima:
        require(errors, value in text, f"{label}: missing attempt minimum {value}")
    require(errors, "9.836" in text and "9.969" in text and "9.884" in text, f"{label}: missing all three fall times")
    terminal_terms = ("终止安全事件", "terminal safety event")
    require(errors, any(term in text for term in terminal_terms), f"{label}: attempt 001 is not identified as terminal")
    deviation_terms = ("偏离协议", "protocol-deviating")
    require(errors, any(term in text for term in deviation_terms), f"{label}: attempts 002/003 are not identified as protocol deviations")
    require(errors, "GENERAL_FAIL" in text, f"{label}: missing GENERAL_FAIL")
    require(errors, not unsupported_comparison(text), f"{label}: unsupported comparative claim")


def validate_plan(errors: list[str], text: str) -> None:
    required_tokens = [
        "legged_wbc/include/legged_wbc/WeightedWbc.h",
        "legged_wbc/src/WeightedWbc.cpp",
        "legged_wbc/CMakeLists.txt",
        "legged_wbc/test/WeightedWbcDiagnosticsTest.cpp",
        "legged_controllers/include/legged_controllers/LeggedController.h",
        "legged_controllers/src/LeggedController.cpp",
        "legged_controllers/scripts/record_myrbf_shadow.py",
        "legged_controllers/launch/myrbf_shadow_gazebo.launch",
        "legged_controllers/CMakeLists.txt",
        "catkin build --no-status legged_wbc legged_controllers",
        "ctest --test-dir build/legged_wbc --output-on-failure",
        "/tmp/myrbf_v5a_safety_solver_diagnosis_20260901",
        "docker.exe",
        "& $dockerExe cp",
        "C:/Users/lenovo/Documents/GO2/results/myrbf_v5a_safety_solver_diagnosis_20260901",
    ]
    for token in required_tokens:
        require(errors, token in text, f"plan: missing required implementation/test/copy token: {token}")
    sequences = re.findall(r"`(BASE\s*→[^`]+)`", text)
    require(errors, len(sequences) >= 5, "plan: too few explicit BASE-first sequences")
    require(errors, all(sequence.strip().startswith("BASE") for sequence in sequences), "plan: a run sequence does not start with BASE")
    require(errors, "每个力级、每个种子都必须先运行 `BASE`" in text, "plan: BASE-first rule is not explicit for every force/seed")
    require(errors, "--output-root C:/" not in text and "--root C:/" not in text, "plan: Windows path is used in a Linux analysis command")
    require(errors, "只执行一次 Docker 复制" in text or "只执行一次 Docker" in text, "plan: one-time checked Docker copy is not explicit")


def load_text(path: Path, errors: list[str], label: str) -> str:
    if not path.is_file():
        errors.append(f"{label}: missing {path}")
        return ""
    return path.read_text(encoding="utf-8")


def collect_validation(root: Path, worktree: Path, summary_override: str | None = None) -> tuple[list[str], dict[str, Any], dict[str, Path]]:
    errors: list[str] = []
    paths = {
        "summary": root / "RESULT_SUMMARY.md",
        "teacher": root / "TEACHER_UPDATE.md",
        "report_check": root / "REPORT_CHECK.json",
        "source_result": worktree / "docs" / "MYRBF_V5A_REPEAT_VALIDATION_RESULT_20260901.md",
        "diagnosis_plan": worktree / "docs" / "superpowers" / "plans" / "2026-09-01-myrbf-v5a-safety-and-solver-diagnosis.md",
        "checker": worktree / "scripts" / "check_myrbf_v5a_repeat_report.py",
        "task5_report": worktree / ".superpowers" / "sdd" / "2026-09-01-myrbf-v5a-repeat-and-constant-diagnosis" / "task-5-report.md",
        "task7_report": worktree / ".superpowers" / "sdd" / "2026-09-01-myrbf-v5a-repeat-and-constant-diagnosis" / "task-7-report.md",
        "progress": worktree / ".superpowers" / "sdd" / "2026-09-01-myrbf-v5a-repeat-and-constant-diagnosis" / "progress.md",
    }

    inventory, inventory_sha = immutable_inventory(root)
    require(errors, len(inventory) == EXPECTED_IMMUTABLE_FILE_COUNT, f"evidence: expected {EXPECTED_IMMUTABLE_FILE_COUNT} immutable files, got {len(inventory)}")
    if EXPECTED_IMMUTABLE_INVENTORY_SHA256:
        require(errors, inventory_sha == EXPECTED_IMMUTABLE_INVENTORY_SHA256, f"evidence: immutable inventory hash mismatch: {inventory_sha}")
    for relative, expected_hash in EXPECTED_HASHES.items():
        path = root / Path(relative)
        require(errors, path.is_file(), f"evidence: missing {relative}")
        if path.is_file():
            require(errors, sha256_file(path) == expected_hash, f"evidence: hash mismatch for {relative}")

    manifest_path = root / "manifest.json"
    manifest: dict[str, Any] = {}
    if manifest_path.is_file():
        manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
        runs = manifest.get("runs", [])
        histories = [attempt for run in runs for attempt in run.get("attempt_history", [])]
        require(errors, len(runs) == 11, f"manifest: expected 11 reached runs, got {len(runs)}")
        require(errors, sum(bool(run.get("complete")) for run in runs) == 10, "manifest: expected 10 complete runs")
        require(errors, len(histories) == 13, f"manifest: expected 13 attempts, got {len(histories)}")
        require(errors, sum(attempt.get("status") == "failed" for attempt in histories) == 3, "manifest: expected 3 failed attempts")
        constant = [run for run in runs if run.get("run_id") == "constant_fy_r01_01_v4"]
        require(errors, len(constant) == 1, "manifest: missing unique constant_fy_r01_01_v4")
        if constant:
            require(errors, constant[0].get("method") == "V4", "manifest: constant failure is not V4")
            require(errors, len(constant[0].get("attempt_history", [])) == 3, "manifest: constant run does not retain three attempts")
        require(errors, manifest.get("protocol_sha256") == EXPECTED_PROTOCOL_SHA256, "manifest: protocol hash mismatch")
        require(errors, manifest.get("source_lock", {}).get("source_commit") == EXPECTED_SOURCE_COMMIT, "manifest: source commit mismatch")

    source_lock_path = root / "SOURCE_LOCK.json"
    if source_lock_path.is_file():
        source_lock = json.loads(source_lock_path.read_text(encoding="utf-8"))
        require(errors, source_lock.get("source_commit") == EXPECTED_SOURCE_COMMIT, "source lock: source commit mismatch")
        require(errors, len(source_lock.get("files", {})) == EXPECTED_SOURCE_LOCK_FILES, "source lock: file count mismatch")

    attempt_facts: dict[str, Any] = {}
    attempt_root = root / "constant_fy_r01_01_v4"
    for attempt_name, expected in EXPECTED_ATTEMPTS.items():
        folder = attempt_root / attempt_name
        try:
            state = read_state_facts(folder / "state.csv")
            wrench = read_wrench_facts(folder / "wrench.csv")
            log_text = (folder / "run.log").read_text(encoding="utf-8", errors="replace")
            errors_count = log_text.count("Maximum number of working set recalculations performed")
        except (OSError, ValueError, csv.Error) as exc:
            errors.append(f"{attempt_name}: {exc}")
            continue
        for key in ("minimum_base_z", "campaign_elapsed_s", "ros_time_s"):
            require(errors, close(float(state[key]), float(expected[key])), f"{attempt_name}: {key} mismatch: {state[key]}")
        require(errors, state["csv_file_row"] == expected["csv_file_row"], f"{attempt_name}: minimum row mismatch: {state['csv_file_row']}")
        require(errors, round(float(state["maximum_state_gap_s"]), 3) == expected["maximum_state_gap_s"], f"{attempt_name}: state gap mismatch: {state['maximum_state_gap_s']}")
        require(errors, round(float(wrench["force_start_s"]), 3) == expected["force_start_s"], f"{attempt_name}: force start mismatch: {wrench['force_start_s']}")
        require(errors, round(float(wrench["force_stop_s"]), 3) == expected["force_stop_s"], f"{attempt_name}: force stop mismatch: {wrench['force_stop_s']}")
        require(errors, wrench["force_start_s"] <= state["campaign_elapsed_s"] <= wrench["force_stop_s"], f"{attempt_name}: minimum is outside 20 N window")
        require(errors, state["minimum_base_z"] < 0.25, f"{attempt_name}: did not cross the frozen 0.25 m safety gate")
        require(errors, errors_count == expected["qp_working_set_errors"], f"{attempt_name}: working-set error count mismatch: {errors_count}")
        attempt_facts[attempt_name] = {**state, **wrench, "qp_working_set_errors": errors_count, "protocol_deviation": expected["protocol_deviation"]}

    for missing in ("metrics.json", "paired_summary.csv", "figures"):
        require(errors, not (root / missing).exists(), f"evidence: forbidden analyzer output exists: {missing}")

    summary = summary_override if summary_override is not None else load_text(paths["summary"], errors, "summary")
    teacher = load_text(paths["teacher"], errors, "teacher")
    source_result = load_text(paths["source_result"], errors, "source result")
    task5 = load_text(paths["task5_report"], errors, "task 5 report")
    task7 = load_text(paths["task7_report"], errors, "task 7 report")
    progress = load_text(paths["progress"], errors, "progress")
    plan = load_text(paths["diagnosis_plan"], errors, "diagnosis plan")
    checker_text = load_text(paths["checker"], errors, "checker")

    require_chronology(errors, "summary", summary)
    require_chronology(errors, "teacher", teacher, compact=True)
    require_chronology(errors, "source result", source_result)
    require_chronology(errors, "task 5 report", task5)
    require_chronology(errors, "task 7 report", task7)
    require_chronology(errors, "progress", progress)
    for label, text in (("summary", summary), ("teacher", teacher), ("source result", source_result), ("task 5 report", task5), ("task 7 report", task7), ("progress", progress)):
        require(errors, "V4" in text and "V5-A" in text, f"{label}: missing V4/V5-A scope statement")
        require(errors, "GENERAL_FAIL" in text, f"{label}: missing decision")
    require(errors, "scripts/check_myrbf_v5a_repeat_report.py" in task7, "task 7 report: retained checker path is missing")
    require(errors, "--mutation-test" in task7, "task 7 report: mutation command is missing")
    require(errors, "只提交三个" in task7 or "exactly three" in task7, "task 7 report: three-file commit scope is missing")
    require(errors, "task7-read-only-v2" not in task7 and "临时文件" not in task7, "task 7 report: stale temporary-checker description remains")
    require(errors, "The third `constant_fy_r01_01_v4` attempt crossed" not in progress, "progress: stale attempt-003 terminal wording remains")
    validate_plan(errors, plan)
    require(errors, CHECKER_NAME in checker_text, "checker: self-identity is missing")

    facts = {
        "decision": "GENERAL_FAIL",
        "terminal_attempt": "attempt-001",
        "protocol_deviating_retries": ["attempt-002", "attempt-003"],
        "attempts": attempt_facts,
        "formal_source_commit": EXPECTED_SOURCE_COMMIT,
        "logical_runs_reached": 11,
        "valid_no_disturbance_runs": 10,
        "formal_attempts": 13,
        "failed_attempts": 3,
        "unstarted_runs": 19,
        "constant_failure_method": "V4",
        "task6": "SKIPPED",
        "v5b": "DISABLED",
        "immutable_file_count": len(inventory),
        "immutable_inventory_sha256": inventory_sha,
        "source_lock_files": EXPECTED_SOURCE_LOCK_FILES,
        "analyzer_outputs_absent": ["metrics.json", "paired_summary.csv", "figures"],
    }
    return errors, facts, paths


def output_payload(facts: dict[str, Any], paths: dict[str, Path]) -> dict[str, Any]:
    hash_keys = ("summary", "teacher", "source_result", "diagnosis_plan", "checker", "task5_report", "task7_report", "progress")
    hashes = {f"{key}_sha256": sha256_file(paths[key]) for key in hash_keys}
    hashes.update({
        "manifest_sha256": EXPECTED_HASHES["manifest.json"],
        "protocol_sha256": EXPECTED_HASHES["PROTOCOL.md"],
        "source_lock_file_sha256": EXPECTED_HASHES["SOURCE_LOCK.json"],
        "locked_inputs_sha256": EXPECTED_HASHES["locked_inputs.sha256"],
        "immutable_inventory_sha256": facts["immutable_inventory_sha256"],
    })
    return {
        "checker": CHECKER_NAME,
        "status": "PASS",
        "claim_checks": {
            "attempt_001_is_terminal_safety_failure": True,
            "attempts_002_003_are_protocol_deviations": True,
            "all_attempts_fell_in_the_20_n_window": True,
            "failure_not_attributed_to_v5a": True,
            "no_v5a_vs_v4_inference_without_pairs": True,
            "base_first_for_every_force_and_seed": True,
            "solver_diagnostics_edits_and_tests_explicit": True,
            "container_evidence_root_and_one_checked_copy": True,
            "retained_checker_mutation_guard": True,
        },
        "verified_facts": facts,
        "hashes": hashes,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True, help="host evidence root")
    parser.add_argument("--worktree", type=Path, required=True, help="source worktree root")
    parser.add_argument("--mutation-test", action="store_true", help="inject an unsupported comparison in memory and require rejection")
    parser.add_argument("--observe-inventory", action="store_true", help="print the current immutable inventory hash without writing REPORT_CHECK.json")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    root = args.root.resolve()
    worktree = args.worktree.resolve()
    if not root.is_dir() or not worktree.is_dir():
        print("FAIL: root and worktree must both exist", file=sys.stderr)
        return 2

    if args.observe_inventory:
        mapping, digest = immutable_inventory(root)
        print(json.dumps({"immutable_file_count": len(mapping), "immutable_inventory_sha256": digest}, indent=2))
        return 0

    errors, facts, paths = collect_validation(root, worktree)
    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1

    summary = paths["summary"].read_text(encoding="utf-8")
    mutation_errors, _, _ = collect_validation(root, worktree, summary_override=summary + "\nV5-A 优于 V4。\n")
    mutation_detected = "summary: unsupported comparative claim" in mutation_errors
    if not mutation_detected:
        print("FAIL: mutation guard did not reject an unsupported comparison", file=sys.stderr)
        return 1

    if args.mutation_test:
        print("MUTATION_GUARD_PASS: summary: unsupported comparative claim")
        return 0

    payload = output_payload(facts, paths)
    report_check = paths["report_check"]
    report_check.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"PASS: {CHECKER_NAME}")
    print(f"terminal_attempt={facts['terminal_attempt']}")
    print(f"protocol_deviations={','.join(facts['protocol_deviating_retries'])}")
    print(f"report_check={report_check}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
