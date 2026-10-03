#!/usr/bin/env python3

import argparse
import csv
from pathlib import Path
import threading

from legged_controllers.myrbf_campaign_timing import (
    CAMPAIGN_START_TOPIC,
    campaign_elapsed,
)


AXES = ("fx_n", "fy_n", "fz_n", "tx_nm", "ty_nm", "tz_nm")


def field_names():
    fields = ["time_s", "dt_s"]
    fields.extend(f"raw_{axis}" for axis in AXES)
    fields.extend(f"filtered_{axis}" for axis in AXES)
    fields.extend(f"estimate_before_{axis}" for axis in AXES)
    fields.extend(f"estimate_after_{axis}" for axis in AXES)
    fields.extend(
        f"feature_residual_{axis[:-2] if axis.endswith('_n') else axis[:-3]}"
        for axis in AXES
    )
    fields.extend(
        ("tracking_sigma_vx", "tracking_sigma_vy", "tracking_sigma_vz",
         "tracking_sigma_wx", "tracking_sigma_wy", "tracking_sigma_wz")
    )
    for prefix in ("actual", "desired"):
        fields.extend(
            (f"{prefix}_base_yaw_rad", f"{prefix}_base_pitch_rad",
             f"{prefix}_base_roll_rad", f"{prefix}_base_x_m",
             f"{prefix}_base_y_m", f"{prefix}_base_z_m")
        )
    fields.extend(("applied_control_fy_n", "control_injection_enabled"))
    fields.extend(("valid_sample_count", "rejected_sample_count"))
    fields.extend(f"net_residual_{axis}" for axis in AXES)
    fields.extend(f"requested_control_{axis}" for axis in AXES)
    fields.extend(f"achieved_control_{axis}" for axis in AXES)
    fields.extend(("allocator_scale", "control_interface_mode"))
    fields.extend(
        f"coverage_{axis[:-2] if axis.endswith('_n') else axis[:-3]}"
        for axis in AXES
    )
    fields.append("update_frozen")
    for prefix in (
        "prediction_derivative_norm",
        "tracking_bias_derivative",
        "leakage_derivative_norm",
        "projection_active",
    ):
        fields.extend(
            f"{prefix}_{axis[:-2] if axis.endswith('_n') else axis[:-3]}"
            for axis in AXES
        )
    fields.append("algorithm_mode")
    for prefix in ("v2_robust_wrench", "v2_robust_bound",
                   "v2_smoothing_width"):
        fields.extend(
            f"{prefix}_{axis[:-2] if axis.endswith('_n') else axis[:-3]}"
            for axis in AXES
        )
    return fields


FIELDS = tuple(field_names())
OUTPUT_FIELDS = ("campaign_start_ros", "wall_elapsed") + FIELDS


def decode_diagnostic(values):
    if len(values) != len(FIELDS):
        raise ValueError(
            f"expected {len(FIELDS)} MyRBF diagnostic values, got {len(values)}"
        )
    return dict(zip(FIELDS, (float(value) for value in values)))


class Recorder:
    def __init__(self, campaign_start_ros, ros_now):
        self._rows = []
        self._lock = threading.Lock()
        self._campaign_start_ros = float(campaign_start_ros)
        self._ros_now = ros_now

    def callback(self, message):
        try:
            row = decode_diagnostic(message.data)
        except ValueError as error:
            import rospy
            rospy.logwarn_throttle(1.0, str(error))
            return
        row["campaign_start_ros"] = self._campaign_start_ros
        row["wall_elapsed"] = campaign_elapsed(
            self._ros_now(), self._campaign_start_ros
        )
        with self._lock:
            if (self._rows and
                    row["wall_elapsed"] == self._rows[-1]["wall_elapsed"]):
                return
            self._rows.append(row)

    def write(self, output):
        output = Path(output)
        output.parent.mkdir(parents=True, exist_ok=True)
        with self._lock:
            rows = list(self._rows)
        with output.open("w", encoding="utf-8", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=OUTPUT_FIELDS)
            writer.writeheader()
            writer.writerows(rows)
        return len(rows)


def main():
    import rospy
    from std_msgs.msg import Float64, Float64MultiArray

    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True)
    parser.add_argument("--duration", type=float, default=0.0)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.duration < 0.0:
        parser.error("--duration must be nonnegative")

    rospy.init_node("myrbf_shadow_recorder")
    campaign_start_ros = rospy.wait_for_message(
        CAMPAIGN_START_TOPIC, Float64, timeout=240.0
    ).data
    recorder = Recorder(campaign_start_ros, rospy.get_time)
    rospy.Subscriber(
        "/myrbf_shadow_diagnostics",
        Float64MultiArray,
        recorder.callback,
        queue_size=1000,
    )

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if (args.duration > 0.0 and
                campaign_elapsed(rospy.get_time(), campaign_start_ros) >= args.duration):
            break
        rate.sleep()

    count = recorder.write(args.output)
    rospy.loginfo("Wrote %d MyRBF shadow samples to %s", count, args.output)


if __name__ == "__main__":
    main()
