#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from analyze_task_c import ensure_complete_pairing


class ExecutedPairingBoundaryTest(unittest.TestCase):
    def test_accepts_unmatched_extra_message_when_analysis_anchors_are_complete(self):
        executed = [{"observation_time": 0.1}, {"observation_time": 0.2}]
        pairing = {"unmatched_messages": [("h_clf", 1.0, "outside_pairing_tolerance", 0.1)]}
        self.assertEqual(ensure_complete_pairing(executed, pairing, [0.1, 0.2], 0.1, 0.2), executed)

    def test_accepts_incomplete_prefix_before_target_window(self):
        executed = [{"observation_time": 0.04}, {"observation_time": 2.0}, {"observation_time": 2.02}]
        pairing = {"unmatched_messages": []}
        self.assertEqual(
            ensure_complete_pairing(executed, pairing, [0.02, 0.04, 2.0, 2.02], 2.0, 2.02),
            executed,
        )

    def test_rejects_missing_anchor_inside_target_analysis_window(self):
        executed = [{"observation_time": 0.1}]
        pairing = {"unmatched_messages": []}
        with self.assertRaises(ValueError):
            ensure_complete_pairing(executed, pairing, [0.1, 0.2], 0.1, 0.2)


if __name__ == "__main__":
    unittest.main()
