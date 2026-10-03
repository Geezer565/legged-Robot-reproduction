#!/usr/bin/env python3
import importlib.util
from pathlib import Path
import tempfile

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module); return module


PROTOCOL = load("d5e_protocol", PACKAGE / "scripts" / "audit_task_d_d5e_protocol.py")
ANALYSIS = load("d5e_analysis", PACKAGE / "scripts" / "analyze_task_d_d5e_feedback_pair.py")


def test_d5e_protocol_changes_only_two_adaptive_switches():
    with tempfile.TemporaryDirectory() as directory:
        assert PROTOCOL.generate(Path(directory) / "protocol") == 0
        import json
        result = json.loads((Path(directory) / "protocol" / "stage_d5e_protocol.json").read_text())
        assert result["passed"]
        assert result["changed_keys"] == ["enableParameterUpdate", "enableAdaptiveDynamics"]
        assert not result["groups"]["on"]["truePlantPayloadInertialEnabled"]
        assert not result["groups"]["off"]["truePlantPayloadInertialEnabled"]


def test_cumulative_trapezoid_preserves_shape_and_integrates_linear_signal():
    time = np.asarray([0.0, 0.5, 1.0])
    values = np.column_stack((time, 2.0 * time))
    result = ANALYSIS.cumulative_trapezoid(time, values)
    assert result.shape == values.shape
    np.testing.assert_allclose(result[-1], [0.5, 1.0])


def test_fairness_requires_exact_source_manifests():
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        common = {"enableParameterUpdate": True, "enableAdaptiveDynamics": True,
                  "truePlantPayloadInertialEnabled": False,
                  "mode_neutral_adaptive_config_sha256": "same", "fixed_files": {"a": "b"}}
        on_path, off_path = root / "on", root / "off"
        on_path.mkdir(); off_path.mkdir()
        (on_path / "source_files.sha256").write_text("same\n")
        (off_path / "source_files.sha256").write_text("same\n")
        off = dict(common); off.update(enableParameterUpdate=False, enableAdaptiveDynamics=False)
        result = ANALYSIS.fair_configs({"config": common, "path": on_path},
                                       {"config": off, "path": off_path})
        assert result["only_adaptive_switches_change"]
        assert result["source_hash_manifest_identical"]
