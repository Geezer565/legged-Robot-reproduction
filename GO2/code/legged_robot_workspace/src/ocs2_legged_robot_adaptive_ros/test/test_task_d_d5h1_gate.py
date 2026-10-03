#!/usr/bin/env python3

import importlib.util
from pathlib import Path

import numpy as np


PACKAGE = Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "d5h1_gate", str(PACKAGE / "scripts" / "audit_task_d_d5h1_gate.py"))
AUDIT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(AUDIT)


def consistent_rows():
    rows = np.zeros((2, 119))
    rows[:, 4:8] = [1.0, 1.0, 1.0, 10.0]
    rows[:, 13:19] = np.asarray([[1, 2, 3, 4, 5, 6],
                                [-1, 1, -2, 2, -3, 3]])
    rows[:, 19:29] = np.arange(20).reshape(2, 10) / 10.0
    y = np.arange(120).reshape(2, 6, 10) / 100.0
    rows[:, 59:119] = y.reshape(2, 60)
    rows[:, 41:47] = np.einsum("nij,nj->ni", y, rows[:, 19:29])
    rows[:, 29:35] = 3.0
    rows[:, 35:41] = 1.0
    rows[:, 47:53] = rows[:, 29:35] - rows[:, 35:41] - rows[:, 41:47]
    rows[:, 53:59] = rows[:, 13:19] * rows[:, 47:53]
    rows[:, 11] = np.sum(rows[:, 53:59], axis=1)
    rows[:, 12] = 0.5 * np.sum(rows[:, 13:19] ** 2, axis=1)
    rows[:, 8] = rows[:, 11] - rows[:, 12]
    rows[:, 9] = rows[:, 8]
    rows[:, 10] = np.maximum(0.0, -rows[:, 8])
    return rows


def test_exact_eq11_decomposition_closes():
    closures = AUDIT.eq11_closures(consistent_rows(), np.eye(6))
    assert max(closures.values()) <= 1.0e-12


def test_eq11_residual_corruption_is_detected():
    rows = consistent_rows()
    rows[0, 47] += 0.25
    closures = AUDIT.eq11_closures(rows, np.eye(6))
    assert closures["residual_wrench"] == 0.25


def test_application_provenance_locks_nominal_only_training():
    provenance = __import__("json").loads(
        (PACKAGE / "config" /
         "d5h1_phase_baseline_application_v1.json").read_text())
    assert provenance["training"]["uncertain_or_payload_data_used"] is False
    assert provenance["default_off"] is True
    assert provenance["startup_gate_used"] is False
    assert provenance["confidence_gate_used"] is False
