#!/usr/bin/env python3
"""D5H0 diagnostic-only replay of a nominal gait-phase update baseline.

The node subscribes to already-published executed diagnostics.  It has no
reference to the estimator, frozen snapshot, Eq. (10), or MPC objects, and it
never publishes to any controller input topic.  The original online update
therefore remains exactly ``Gamma * Y.T * sigma``.
"""

import csv
import hashlib
import json
import math
import threading
from pathlib import Path

import numpy as np


PARAMETERS = ("m", "hx", "hy", "hz", "Jxx", "Jxy", "Jxz",
              "Jyy", "Jyz", "Jzz")
EXPECTED_MODES = (13, 7, 14, 11)
INPUT_LABEL = "payload_inertial_10d_v3_118_fields"
OUTPUT_LABEL = "payload_inertial_d5h0_shadow_v1_65_fields"
OUTPUT_FIELDS = 65


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_phase_baseline(path, phase_bins=24):
    """Load the exact D5G long-form CSV into a [phase_bin, parameter] table."""
    path = Path(path)
    values = np.full((phase_bins, len(PARAMETERS)), np.nan)
    durations = np.full(phase_bins, np.nan)
    phase_bounds = np.full((phase_bins, 2), np.nan)
    parameter_index = {name: index for index, name in enumerate(PARAMETERS)}
    required = {"phase_bin", "phase_start", "phase_end", "training_duration_s",
                "parameter", "nominal_baseline_drive_rate"}
    with path.open(newline="") as stream:
        reader = csv.DictReader(stream)
        if set(reader.fieldnames or ()) != required:
            raise ValueError("D5H0 baseline CSV schema mismatch")
        for row in reader:
            phase = int(row["phase_bin"])
            parameter = row["parameter"]
            if phase < 0 or phase >= phase_bins or parameter not in parameter_index:
                raise ValueError("D5H0 baseline phase/parameter is invalid")
            column = parameter_index[parameter]
            if np.isfinite(values[phase, column]):
                raise ValueError("D5H0 baseline contains a duplicate cell")
            values[phase, column] = float(row["nominal_baseline_drive_rate"])
            duration = float(row["training_duration_s"])
            bounds = (float(row["phase_start"]), float(row["phase_end"]))
            if np.isfinite(durations[phase]) and not math.isclose(
                    durations[phase], duration, rel_tol=0.0, abs_tol=1.0e-12):
                raise ValueError("D5H0 baseline duration differs within one bin")
            if np.isfinite(phase_bounds[phase, 0]) and not np.allclose(
                    phase_bounds[phase], bounds, rtol=0.0, atol=1.0e-12):
                raise ValueError("D5H0 baseline bounds differ within one bin")
            durations[phase] = duration
            phase_bounds[phase] = bounds
    if not np.all(np.isfinite(values)) or not np.all(np.isfinite(durations)):
        raise ValueError("D5H0 baseline is incomplete or nonfinite")
    if np.any(durations <= 0.0):
        raise ValueError("D5H0 baseline contains nonpositive training duration")
    expected_bounds = np.column_stack((
        np.arange(phase_bins, dtype=float) / phase_bins,
        np.arange(1, phase_bins + 1, dtype=float) / phase_bins))
    if not np.allclose(phase_bounds, expected_bounds, rtol=0.0, atol=1.0e-12):
        raise ValueError("D5H0 baseline phase bounds are not a complete partition")
    return {
        "values": values,
        "durations": durations,
        "phase_bounds": phase_bounds,
        "sha256": sha256(path),
        "path": str(path.resolve()),
    }


class StaticWalkPhaseTracker(object):
    """Causal tracker matching the D5G 13->7->14->11->13 cycle convention."""

    def __init__(self, cycle_duration=1.2, phase_bins=24,
                 minimum_cycle_start=2.2, expected_modes=EXPECTED_MODES):
        if not math.isfinite(cycle_duration) or cycle_duration <= 0.0:
            raise ValueError("cycle_duration must be finite and positive")
        if phase_bins <= 0 or phase_bins % len(expected_modes) != 0:
            raise ValueError("phase_bins must be a positive multiple of mode count")
        self.cycle_duration = float(cycle_duration)
        self.phase_bins = int(phase_bins)
        self.minimum_cycle_start = float(minimum_cycle_start)
        self.expected_modes = tuple(int(value) for value in expected_modes)
        self.previous_mode = None
        self.sequence_index = -1
        self.cycle_start = None
        self.sequence_valid = False
        self.last_time = None

    def _invalid(self, time, mode):
        return {
            "time": float(time), "mode": int(mode), "phase_valid": False,
            "steady_active": False, "normalized_phase": -1.0,
            "phase_bin": -1, "cycle_boundary": False,
            "cycle_start": -1.0, "cycle_duration": self.cycle_duration,
        }

    def _start_cycle(self, time):
        self.cycle_start = float(time)
        self.sequence_index = 0
        self.sequence_valid = True

    def sample(self, time, mode):
        time, mode = float(time), int(mode)
        if not math.isfinite(time):
            raise ValueError("phase sample time must be finite")
        if self.last_time is not None and time <= self.last_time:
            raise ValueError("phase samples must be strictly increasing")

        mode_changed = self.previous_mode is not None and mode != self.previous_mode
        close_previous_cycle = bool(
            mode_changed and self.sequence_valid and
            self.previous_mode == self.expected_modes[-1] and
            mode == self.expected_modes[0])

        if close_previous_cycle:
            # D5G explicit-Euler intervals include the transition endpoint in
            # the preceding cycle (phase just below one).  Reset the causal
            # tracker only after emitting that endpoint.
            old_start = self.cycle_start
            result = {
                "time": time, "mode": mode, "phase_valid": True,
                "steady_active": old_start >= self.minimum_cycle_start,
                "normalized_phase": float(np.nextafter(1.0, 0.0)),
                "phase_bin": self.phase_bins - 1, "cycle_boundary": True,
                "cycle_start": old_start,
                "cycle_duration": self.cycle_duration,
            }
            self._start_cycle(time)
            self.previous_mode = mode
            self.last_time = time
            return result

        if self.previous_mode is None:
            if mode == self.expected_modes[0]:
                self._start_cycle(time)
            else:
                self.sequence_valid = False
        elif mode_changed:
            expected_next = (self.sequence_index + 1
                             if self.sequence_valid else -1)
            if (self.sequence_valid and expected_next < len(self.expected_modes)
                    and mode == self.expected_modes[expected_next]):
                self.sequence_index = expected_next
            elif mode == self.expected_modes[0]:
                self._start_cycle(time)
            else:
                self.sequence_valid = False
                self.sequence_index = -1
                self.cycle_start = None

        result = self._invalid(time, mode)
        if self.sequence_valid and self.cycle_start is not None:
            phase = (time - self.cycle_start) / self.cycle_duration
            if 0.0 <= phase < 1.0:
                phase_bin = min(int(math.floor(phase * self.phase_bins)),
                                self.phase_bins - 1)
                bins_per_mode = self.phase_bins // len(self.expected_modes)
                phase_mode_index = min(phase_bin // bins_per_mode,
                                       len(self.expected_modes) - 1)
                mode_boundary = self.sequence_index / float(len(self.expected_modes))
                rounding_at_valid_transition = (
                    abs(phase - mode_boundary) <= 1.0e-9)
                if ((phase_mode_index == self.sequence_index or
                     rounding_at_valid_transition) and
                        mode == self.expected_modes[self.sequence_index]):
                    result.update({
                        "phase_valid": True,
                        "steady_active": self.cycle_start >= self.minimum_cycle_start,
                        "normalized_phase": phase,
                        "phase_bin": phase_bin,
                        "cycle_start": self.cycle_start,
                    })

        self.previous_mode = mode
        self.last_time = time
        return result


class ShadowAccumulator(object):
    """Integrate two candidate derivatives without exposing them to control."""

    def __init__(self, gamma):
        self.gamma = np.asarray(gamma, dtype=float)
        if self.gamma.shape != (10, 10) or not np.all(np.isfinite(self.gamma)):
            raise ValueError("D5H0 Gamma must be a finite 10x10 matrix")
        self.raw = np.zeros(10)
        self.corrected = np.zeros(10)
        self.last_sample_time = None

    def update(self, time, active, cycle_start, raw_drive, corrected_drive):
        time = float(time)
        raw_drive = np.asarray(raw_drive, dtype=float)
        corrected_drive = np.asarray(corrected_drive, dtype=float)
        if raw_drive.shape != (10,) or corrected_drive.shape != (10,):
            raise ValueError("D5H0 drive vectors must contain ten elements")
        dt = 0.0
        if active:
            if self.last_sample_time is not None:
                dt = time - self.last_sample_time
            elif math.isfinite(cycle_start) and time > cycle_start:
                dt = time - cycle_start
            if dt < 0.0 or not math.isfinite(dt):
                raise ValueError("D5H0 integration interval is invalid")
            self.raw += dt * self.gamma.dot(raw_drive)
            self.corrected += dt * self.gamma.dot(corrected_drive)
        self.last_sample_time = time
        return dt, self.raw.copy(), self.corrected.copy()


def shadow_sample(row, mode, baseline, phase_tracker, accumulator):
    """Build one finite, fixed-width D5H0 diagnostic row from a v3 sample."""
    row = np.asarray(row, dtype=float)
    if row.shape != (118,) or not np.all(np.isfinite(row)):
        raise ValueError("D5H0 requires one finite 118-field v3 sample")
    sigma = row[42:48]
    regressor = row[48:108].reshape(6, 10)
    raw_drive = regressor.T.dot(sigma)
    phase = phase_tracker.sample(row[0], mode)
    baseline_drive = np.zeros(10)
    if phase["steady_active"]:
        baseline_drive = baseline[phase["phase_bin"]].copy()
    corrected_drive = raw_drive - baseline_drive
    dt, cumulative_raw, cumulative_corrected = accumulator.update(
        row[0], phase["steady_active"], phase["cycle_start"],
        raw_drive, corrected_drive)
    result = np.asarray([
        row[0], row[1], row[2], row[3], float(mode),
        1.0 if phase["phase_valid"] else 0.0,
        1.0 if phase["steady_active"] else 0.0,
        phase["normalized_phase"], float(phase["phase_bin"]),
        1.0 if phase["cycle_boundary"] else 0.0,
        phase["cycle_start"], phase["cycle_duration"], dt, 1.0, 1.0,
    ] + raw_drive.tolist() + baseline_drive.tolist() +
        corrected_drive.tolist() + cumulative_raw.tolist() +
        cumulative_corrected.tolist(), dtype=float)
    if result.shape != (OUTPUT_FIELDS,) or not np.all(np.isfinite(result)):
        raise RuntimeError("D5H0 output schema is not finite and fixed-width")
    return result


class D5H0ShadowNode(object):
    def __init__(self, rospy, Float64MultiArray, MultiArrayDimension, String,
                 Observation):
        self.rospy = rospy
        baseline_file = rospy.get_param("~baseline_file")
        phase_bins = int(rospy.get_param("~phase_bins", 24))
        cycle_duration = float(rospy.get_param("~cycle_duration", 1.2))
        minimum_cycle_start = float(
            rospy.get_param("~minimum_cycle_start_time", 2.2))
        gamma_diagonal = np.asarray(
            rospy.get_param("~gamma_diagonal", [1.0] * 10), dtype=float)
        if gamma_diagonal.shape != (10,) or not np.all(np.isfinite(gamma_diagonal)):
            raise ValueError("~gamma_diagonal must contain ten finite values")
        self.baseline = load_phase_baseline(baseline_file, phase_bins)
        self.phase_tracker = StaticWalkPhaseTracker(
            cycle_duration, phase_bins, minimum_cycle_start)
        self.accumulator = ShadowAccumulator(np.diag(gamma_diagonal))
        self.Float64MultiArray = Float64MultiArray
        self.MultiArrayDimension = MultiArrayDimension
        self.lock = threading.RLock()
        self.modes = {}
        self.pending = {}
        self.last_processed_time = -math.inf
        self.publisher = rospy.Publisher(
            "/task_d/d5h0_shadow_update_sample", Float64MultiArray,
            queue_size=20)
        self.identity_publisher = rospy.Publisher(
            "/task_d/d5h0_shadow_identity", String, queue_size=1,
            latch=True)
        identity = {
            "schema_version": 1,
            "stage": "D5H0_shadow_diagnostics",
            "shadow_only": True,
            "controller_write_path": False,
            "input_topic": "/task_d/payload_inertial_10d_v3_sample",
            "output_topic": "/task_d/d5h0_shadow_update_sample",
            "output_label": OUTPUT_LABEL,
            "output_fields": OUTPUT_FIELDS,
            "baseline_file": self.baseline["path"],
            "baseline_sha256": self.baseline["sha256"],
            "phase_bins": phase_bins,
            "minimum_cycle_start_time": minimum_cycle_start,
            "cycle_duration": cycle_duration,
            "expected_modes": list(EXPECTED_MODES),
            "gamma_diagonal": gamma_diagonal.tolist(),
            "actual_estimator_formula_unchanged": "pi_dot=Gamma*Y^T*sigma",
            "shadow_formula": "corrected_drive=Y^T*sigma-nominal_phase_baseline",
        }
        self.identity_publisher.publish(String(data=json.dumps(
            identity, sort_keys=True)))
        self.observation_subscriber = rospy.Subscriber(
            "/legged_robot_mpc_observation", Observation,
            self._observation_callback, queue_size=100)
        self.estimate_subscriber = rospy.Subscriber(
            "/task_d/payload_inertial_10d_v3_sample", Float64MultiArray,
            self._estimate_callback, queue_size=100)

    @staticmethod
    def _time_key(time):
        return int(round(float(time) * 1000000.0))

    def _observation_callback(self, message):
        time = float(message.time)
        key = self._time_key(time)
        with self.lock:
            self.modes[key] = (time, int(message.mode))
            pending = self.pending.pop(key, None)
            if pending is not None:
                self._process_locked(pending, int(message.mode))
            self._prune_locked(time)

    def _estimate_callback(self, message):
        label = message.layout.dim[0].label if message.layout.dim else ""
        if label != INPUT_LABEL or len(message.data) != 118:
            self.rospy.logwarn_throttle(
                5.0, "D5H0 ignored a non-v3 or malformed input sample")
            return
        row = np.asarray(message.data, dtype=float)
        key = self._time_key(row[0])
        with self.lock:
            mode = self.modes.get(key)
            if mode is None or abs(mode[0] - row[0]) > 1.0e-6:
                self.pending[key] = row
                return
            self._process_locked(row, mode[1])
            self._prune_locked(row[0])

    def _process_locked(self, row, mode):
        if row[0] <= self.last_processed_time:
            return
        try:
            output = shadow_sample(row, mode, self.baseline["values"],
                                   self.phase_tracker, self.accumulator)
        except Exception as error:
            self.rospy.logerr("D5H0 shadow sample rejected: %s", error)
            return
        message = self.Float64MultiArray()
        dimension = self.MultiArrayDimension()
        dimension.label = OUTPUT_LABEL
        dimension.size = OUTPUT_FIELDS
        dimension.stride = OUTPUT_FIELDS
        message.layout.dim = [dimension]
        message.data = output.tolist()
        self.publisher.publish(message)
        self.last_processed_time = float(row[0])

    def _prune_locked(self, current_time):
        cutoff = float(current_time) - 2.0
        for collection in (self.modes, self.pending):
            stale = [key for key in collection
                     if key / 1000000.0 < cutoff]
            for key in stale:
                collection.pop(key, None)


def main():
    import rospy
    from ocs2_msgs.msg import mpc_observation
    from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String

    rospy.init_node("task_d_d5h0_shadow_diagnostics")
    D5H0ShadowNode(rospy, Float64MultiArray, MultiArrayDimension, String,
                   mpc_observation)
    rospy.loginfo("D5H0 shadow diagnostics active; controller update is unchanged")
    rospy.spin()


if __name__ == "__main__":
    main()
