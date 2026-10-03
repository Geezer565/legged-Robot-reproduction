#!/usr/bin/env python3

import importlib.util
import pathlib
import tempfile
import unittest


PACKAGE = pathlib.Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODE = load("d5i4_mode", PACKAGE / "scripts" / "set_task_d_d5i4_mode.py")


class D5I4SmokeTest(unittest.TestCase):
    @unittest.skipUnless(
        MODE.BASELINE.exists() and MODE.PROVENANCE.exists(),
        "D5I3 frozen artifacts were not included in the source handoff",
    )
    def test_four_modes_are_full_adaptive_e3_and_only_two_variables_change(self):
        base = MODE.BASE_CONFIG.read_text()
        configs = {}
        for group in MODE.GROUPS:
            text = MODE.materialize(base, group)
            configs[group] = MODE.effective_config(text, group)
            self.assertTrue(configs[group]["enableParameterUpdate"])
            self.assertTrue(configs[group]["enableAdaptiveDynamics"])
            self.assertTrue(configs[group]["enableClfConstraint"])
            self.assertTrue(configs[group]["enableEncodedReferenceDerivatives"])
            self.assertEqual(configs[group]["candidate"], "E3_combined")
            self.assertFalse(configs[group][
                "compositePhaseBaselineTrainingUsesPayloadData"])
        self.assertFalse(configs["nominal_original"][
            "enableCompositePhaseBaselineCorrection"])
        self.assertTrue(configs["nominal_phase_corrected"][
            "enableCompositePhaseBaselineCorrection"])
        self.assertFalse(configs["nominal_original"][
            "truePlantPayloadInertialEnabled"])
        self.assertTrue(configs["payload_original"][
            "truePlantPayloadInertialEnabled"])

    @unittest.skipUnless(
        MODE.BASELINE.exists() and MODE.PROVENANCE.exists(),
        "D5I3 frozen artifacts were not included in the source handoff",
    )
    def test_frozen_artifact_hash_and_provenance_are_enforced(self):
        MODE.validate_frozen_artifacts(MODE.BASELINE, MODE.PROVENANCE)
        with tempfile.TemporaryDirectory() as directory:
            bad = pathlib.Path(directory) / "baseline.csv"
            bad.write_text(MODE.BASELINE.read_text() + "\n")
            with self.assertRaisesRegex(RuntimeError, "baseline hash"):
                MODE.validate_frozen_artifacts(bad, MODE.PROVENANCE)

    def test_runtime_topic_and_default_off_source_contract(self):
        node = (PACKAGE / "src" / "LeggedRobotAdaptiveDdpMpcNode.cpp").read_text()
        base = MODE.BASE_CONFIG.read_text()
        self.assertIn("/task_d/d5i4_composite_update_sample", node)
        self.assertIn("payload_inertial_d5i4_composite_update_v1_76_fields", node)
        self.assertNotIn("enableCompositePhaseBaselineCorrection", base)


if __name__ == "__main__":
    unittest.main()
