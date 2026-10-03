#!/usr/bin/env python3

import importlib.util
import json
import tempfile
import unittest
from argparse import Namespace
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "set_task_d_mode.py"
SPEC = importlib.util.spec_from_file_location("set_task_d_mode", str(SCRIPT))
MODE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODE)


class TaskDModeAuditTest(unittest.TestCase):
    def setUp(self):
        self.base = MODE.BASE_CONFIG.read_text()

    def test_mode_neutral_controller_config_is_identical(self):
        texts = {group: MODE.materialize_text(self.base, group)
                 for group in MODE.GROUPS}
        self.assertEqual(1, len({MODE.neutral_text(text) for text in texts.values()}))

    def test_pairwise_independent_variables_are_exact(self):
        configs = {group: MODE.effective_config(
            MODE.materialize_text(self.base, group), group)
            for group in MODE.GROUPS}
        keys = list(MODE.MODE_KEYS) + ["truePlantPayloadInertialEnabled"]
        expected = {
            ("nominal", "uncertain_adaptive"):
                ["truePlantPayloadInertialEnabled"],
            ("uncertain_off", "uncertain_adaptive"):
                ["enableParameterUpdate", "enableAdaptiveDynamics"],
            ("nominal", "uncertain_off"):
                ["enableParameterUpdate", "enableAdaptiveDynamics",
                 "truePlantPayloadInertialEnabled"],
        }
        for pair, wanted in expected.items():
            left, right = pair
            changed = [key for key in keys
                       if configs[left][key] != configs[right][key]]
            self.assertEqual(wanted, changed)

    def test_audit_artifacts_report_pass(self):
        with tempfile.TemporaryDirectory() as directory:
            status = MODE.audit(Namespace(
                base_config=MODE.BASE_CONFIG, output_dir=Path(directory)))
            result = json.loads((Path(directory) / "fairness_audit.json").read_text())
            self.assertEqual(0, status)
            self.assertTrue(result["passed"])
            self.assertFalse(result["formal_campaign_started"])


if __name__ == "__main__":
    unittest.main()
