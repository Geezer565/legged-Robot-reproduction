#!/usr/bin/env python3
"""Record the controller's self-described diagnostic topic without guessing order."""
import argparse
import csv
import json
import math
from pathlib import Path
import threading
import time
import sys

TOPIC = "/myrbf_v2_lyapunov_diagnostics"
COLUMNS_PARAM = "/myrbf_v2_lyapunov_columns"
PREFIX = ("campaign_start_ros", "wall_elapsed", "received_ros_time")


def validate_columns(columns):
    if (not isinstance(columns, list) or not columns or
            any(not isinstance(c, str) or not c.strip() for c in columns) or
            len(set(columns)) != len(columns) or set(columns).intersection(PREFIX)):
        raise ValueError("controller columns must be unique nonempty strings, without recorder fields")
    return tuple(columns)


def decode(columns, values):
    if len(columns) != len(values):
        raise ValueError("diagnostic length differs from controller column schema")
    converted = [float(v) for v in values]
    if not all(math.isfinite(v) for v in converted):
        raise ValueError("diagnostic contains nonfinite values")
    return converted


def main(argv=None):
    import rospy
    from std_msgs.msg import Float64, Float64MultiArray
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--duration", type=float, default=18.0)
    p.add_argument("--topic", default=TOPIC)
    p.add_argument("--columns-param", default=COLUMNS_PARAM)
    args = p.parse_args(rospy.myargv(argv=argv)[1:])
    if not math.isfinite(args.duration) or args.duration <= 0:
        p.error("duration must be finite and positive")
    rospy.init_node("adaptive_wrench_recorder")
    deadline = time.monotonic() + 240
    while not rospy.has_param(args.columns_param):
        if rospy.is_shutdown() or time.monotonic() > deadline:
            raise RuntimeError("controller did not publish " + args.columns_param)
        time.sleep(0.05)
    columns = validate_columns(rospy.get_param(args.columns_param))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    schema_path = args.output.with_suffix(".schema.json")
    schema_path.write_text(json.dumps({"topic": args.topic, "columns_param": args.columns_param,
                                      "columns": columns, "recorder_prefix": PREFIX}, indent=2) + "\n")
    start = rospy.wait_for_message("/myrbf_campaign_start", Float64, timeout=240).data
    state = {"rows": 0, "errors": [], "done": False}
    lock = threading.Lock()
    with args.output.open("x", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(PREFIX + columns)
        handle.flush()

        def receive(msg):
            now = rospy.get_time()
            elapsed = now - start
            if elapsed < 0 or elapsed > args.duration:
                return
            with lock:
                if state["done"]:
                    return
                # Preserve all values, even invalid rows, for failure investigation.
                writer.writerow((start, elapsed, now, *msg.data))
                handle.flush()
                state["rows"] += 1
                try:
                    decode(columns, msg.data)
                except ValueError as error:
                    if len(state["errors"]) < 20:
                        state["errors"].append(str(error))
                    rospy.logerr_throttle(1.0, "Adaptive diagnostic data failure: %s", str(error))

        sub = rospy.Subscriber(args.topic, Float64MultiArray, receive, queue_size=1000)
        rate = rospy.Rate(50)
        try:
            while not rospy.is_shutdown() and rospy.get_time() - start < args.duration:
                if tuple(rospy.get_param(args.columns_param)) != columns:
                    state["errors"].append("controller schema changed during recording")
                    break
                rate.sleep()
        finally:
            with lock:
                state["done"] = True
            sub.unregister()
    if rospy.get_time() - start < args.duration - .05:
        state["errors"].append("recorder stopped before the requested experiment duration")
    args.output.with_suffix(".status.json").write_text(json.dumps(state, indent=2) + "\n")
    if not state["rows"] or state["errors"]:
        raise RuntimeError("invalid or empty adaptive diagnostics; inspect retained CSV/status")


if __name__ == "__main__":
    try:
        main()
    except BaseException as error:
        # Declare startup/schema/topic/I/O failures even before a CSV exists.
        if "--output" in sys.argv:
            output = Path(sys.argv[sys.argv.index("--output") + 1])
            status_path = output.with_suffix(".status.json")
            try:
                state = json.loads(status_path.read_text()) if status_path.exists() else {"rows": 0, "errors": []}
                state["errors"].append(type(error).__name__ + ": " + str(error))
                state["fatal"] = True
                status_path.parent.mkdir(parents=True, exist_ok=True)
                status_path.write_text(json.dumps(state, indent=2) + "\n")
            except OSError:
                pass  # stderr/run.log still records an unwritable output error.
        raise
