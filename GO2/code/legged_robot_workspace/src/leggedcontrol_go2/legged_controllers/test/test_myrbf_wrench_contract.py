#!/usr/bin/env python3
from pathlib import Path
import importlib.util
import unittest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/check_myrbf_wrench_contract.py"
SPEC = importlib.util.spec_from_file_location("wrench_contract", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class WrenchContractTest(unittest.TestCase):
    def test_controller_wbc_round_trip(self):
        MODULE.run()


if __name__ == "__main__":
    unittest.main()
