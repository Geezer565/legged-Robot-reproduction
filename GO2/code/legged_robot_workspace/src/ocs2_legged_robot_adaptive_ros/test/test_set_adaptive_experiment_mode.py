#!/usr/bin/env python3
import importlib.util
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).resolve().parents[1] / "scripts" / "set_adaptive_experiment_mode.py"
SPEC = importlib.util.spec_from_file_location("set_adaptive_experiment_mode", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


BASE = """adaptive_clf_mpc
{
  enableParameterUpdate  true
  enableAdaptiveDynamics true
  enableClfConstraint     true
  enableInputBiasCost     false
  softConstraintMu        0.1
}

adaptive_estimator
{
  enableParameterUpdate false
}
"""


class ExperimentModeTest(unittest.TestCase):
    def test_all_formal_modes_set_exact_three_switches(self):
        expected = {
            "nominal": ("false", "false", "false"),
            "estimate_only": ("true", "false", "false"),
            "full_adaptive": ("true", "true", "false"),
            "clf_off": ("true", "true", "false"),
            "clf_on": ("true", "true", "true"),
        }
        for mode, values in expected.items():
            updated = MODULE.rewrite_config_text(BASE, mode)
            self.assertEqual(MODULE.read_active_switches(updated), dict(zip(MODULE.SWITCHES, values)))
            self.assertIn("adaptive_estimator\n{\n  enableParameterUpdate false", updated)
            self.assertIn("enableInputBiasCost     false", updated)
            self.assertIn("softConstraintMu        0.1", updated)

    def test_rewrite_is_idempotent(self):
        once = MODULE.rewrite_config_text(BASE, "estimate_only")
        self.assertEqual(MODULE.rewrite_config_text(once, "estimate_only"), once)

    def test_missing_or_duplicate_active_switch_is_rejected(self):
        missing = BASE.replace("  enableAdaptiveDynamics true\n", "")
        duplicate = BASE.replace(
            "  enableClfConstraint     true\n",
            "  enableClfConstraint     true\n  enableClfConstraint false\n",
        )
        with self.assertRaisesRegex(ValueError, "enableAdaptiveDynamics"):
            MODULE.rewrite_config_text(missing, "nominal")
        with self.assertRaisesRegex(ValueError, "enableClfConstraint"):
            MODULE.rewrite_config_text(duplicate, "nominal")

    def test_unknown_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "unknown mode"):
            MODULE.rewrite_config_text(BASE, "mystery")


if __name__ == "__main__":
    unittest.main()
