#!/usr/bin/env python3
"""Create the two-mode fairness protocol for D5E nominal walking."""

import argparse
import difflib
import importlib.util
import json
import re
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
MODE_SCRIPT = PACKAGE / "scripts" / "set_task_d_mode.py"
SPEC = importlib.util.spec_from_file_location("task_d_modes", str(MODE_SCRIPT))
MODES = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODES)
MODE_KEYS = ("enableParameterUpdate", "enableAdaptiveDynamics")


def replace_mode(text, enabled):
    result = text
    for key in MODE_KEYS:
        replacement = "true" if enabled else "false"
        pattern = r"^(\s*" + re.escape(key) + r"\s+)(true|false)(\s*(?:;.*)?$)"
        result, count = re.subn(pattern, r"\g<1>" + replacement + r"\g<3>",
                                result, flags=re.MULTILINE)
        if count != 1:
            raise RuntimeError("mode key must occur exactly once: " + key)
    MODES.validate_base(result)
    return result


def generate(output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    base = MODES.BASE_CONFIG.read_text()
    texts = {"on": replace_mode(base, True), "off": replace_mode(base, False)}
    configs = {}
    for name, text in texts.items():
        config = MODES.effective_config(text, "nominal")
        config.update({
            "schema_version": 1,
            "stage": "D5E_nominal_walking_adaptation_feedback",
            "group": name,
            "truePlantPayloadInertialEnabled": False,
            "experiment_condition": "walk_target",
            "acquisition_end_mpc_time_s": 8.0,
            "common_grid_dt_s": 0.02,
        })
        configs[name] = config
        (output / (name + ".info")).write_text(text)
        (output / (name + ".json")).write_text(
            json.dumps(config, indent=2, sort_keys=True) + "\n")

    changed = [key for key in MODE_KEYS
               if configs["on"][key] != configs["off"][key]]
    checks = {
        "only_two_adaptive_switches_change": changed == list(MODE_KEYS),
        "mode_neutral_config_identical":
            configs["on"]["mode_neutral_adaptive_config_sha256"] ==
            configs["off"]["mode_neutral_adaptive_config_sha256"],
        "fixed_controller_files_identical":
            configs["on"]["fixed_files"] == configs["off"]["fixed_files"],
        "nominal_plant_both_groups": not any(
            config["truePlantPayloadInertialEnabled"] for config in configs.values()),
        "target_and_gait_identical": all(
            configs["on"][key] == configs["off"][key]
            for key in ("target", "gait", "experiment_condition")),
    }
    passed = all(checks.values())
    protocol = {
        "schema_version": 1,
        "stage": "D5E_nominal_walking_adaptation_feedback_protocol",
        "passed": passed,
        "scientific_question":
            "Does adaptation feedback amplify nominal walking velocity residuals?",
        "independent_variables": list(MODE_KEYS),
        "changed_keys": changed,
        "checks": checks,
        "groups": configs,
        "interpretation_boundary":
            "One paired diagnostic run per mode is mechanism evidence, not a formal statistical campaign.",
    }
    (output / "stage_d5e_protocol.json").write_text(
        json.dumps(protocol, indent=2, sort_keys=True) + "\n")
    diff = difflib.unified_diff(
        texts["on"].splitlines(True), texts["off"].splitlines(True),
        fromfile="on.info", tofile="off.info")
    (output / "effective_config_diff.txt").write_text("".join(diff))
    print(json.dumps(protocol, indent=2, sort_keys=True))
    return 0 if passed else 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output_directory")
    args = parser.parse_args()
    return generate(args.output_directory)


if __name__ == "__main__":
    raise SystemExit(main())
