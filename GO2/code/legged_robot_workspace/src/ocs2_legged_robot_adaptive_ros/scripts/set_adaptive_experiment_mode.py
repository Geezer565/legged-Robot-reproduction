#!/usr/bin/env python3
"""Atomically select a formal Task B or Task C experiment mode."""

import argparse
import json
import re
import sys
from pathlib import Path


ACTIVE_SECTION = "adaptive_clf_mpc"
SWITCHES = (
    "enableParameterUpdate",
    "enableAdaptiveDynamics",
    "enableClfConstraint",
)
MODE_SWITCHES = {
    "nominal": ("false", "false", "false"),
    "estimate_only": ("true", "false", "false"),
    "full_adaptive": ("true", "true", "false"),
    "clf_off": ("true", "true", "false"),
    "clf_on": ("true", "true", "true"),
}
DEFAULT_CONFIG = (
    Path(__file__).resolve().parents[2]
    / "ocs2_legged_robot_adaptive"
    / "config"
    / "adaptive_settings.info"
)


def _active_section_bounds(text):
    match = re.search(r"(?m)^\s*" + re.escape(ACTIVE_SECTION) + r"\s*\{", text)
    if not match:
        raise ValueError("missing adaptive_clf_mpc section")
    opening = text.find("{", match.start(), match.end())
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return match.start(), index + 1
    raise ValueError("unterminated adaptive_clf_mpc section")


def _read_switch(section, key):
    matches = re.findall(
        r"^\s*" + re.escape(key) + r"\s+(true|false)\s*(?:;.*)?$",
        section,
        flags=re.MULTILINE,
    )
    if len(matches) != 1:
        raise ValueError("{} must appear exactly once in adaptive_clf_mpc".format(key))
    return matches[0]


def read_active_switches(text):
    begin, end = _active_section_bounds(text)
    section = text[begin:end]
    return {key: _read_switch(section, key) for key in SWITCHES}


def rewrite_config_text(text, mode):
    if mode not in MODE_SWITCHES:
        raise ValueError("unknown mode: {}".format(mode))
    begin, end = _active_section_bounds(text)
    section = text[begin:end]
    for key in SWITCHES:
        _read_switch(section, key)

    updated = section
    for key, value in zip(SWITCHES, MODE_SWITCHES[mode]):
        pattern = re.compile(
            r"^(\s*" + re.escape(key) + r"\s+)(true|false)(\s*(?:;.*)?$)",
            flags=re.MULTILINE,
        )
        updated, count = pattern.subn(lambda match: match.group(1) + value + match.group(3), updated)
        if count != 1:
            raise ValueError("{} must appear exactly once in adaptive_clf_mpc".format(key))

    result = text[:begin] + updated + text[end:]
    expected = dict(zip(SWITCHES, MODE_SWITCHES[mode]))
    if read_active_switches(result) != expected:
        raise RuntimeError("experiment mode verification failed")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description="Select a formal adaptive experiment mode")
    parser.add_argument("mode", choices=tuple(MODE_SWITCHES))
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args(argv)

    original = args.config.read_text(encoding="utf-8")
    updated = rewrite_config_text(original, args.mode)
    if not args.dry_run:
        temporary = args.config.with_name(args.config.name + ".tmp")
        temporary.write_text(updated, encoding="utf-8")
        temporary.replace(args.config)

    print(json.dumps({
        "config": str(args.config),
        "mode": args.mode,
        "switches": read_active_switches(updated),
        "written": not args.dry_run,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print("set_adaptive_experiment_mode.py: {}".format(error), file=sys.stderr)
        raise SystemExit(1)
