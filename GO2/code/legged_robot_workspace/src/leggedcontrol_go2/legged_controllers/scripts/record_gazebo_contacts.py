#!/usr/bin/env python3

import argparse
import csv
import math
import time
from pathlib import Path

import rospy
from gazebo_msgs.msg import ContactsState


FEET = ["FR", "FL", "RR", "RL"]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--output", required=True)
    p.add_argument("--duration", type=float, required=True)
    p.add_argument("--rate", type=float, default=100.0)
    p.add_argument("--stale-time", type=float, default=0.05)
    return p.parse_args(rospy.myargv()[1:])


class ContactRecorder:
    def __init__(self, args):
        self.args = args

        # 每只脚缓存：
        # [Fx,Fy,Fz,Tx,Ty,Tz]
        self.values = {
            foot: [0.0] * 6
            for foot in FEET
        }

        self.last_wall = {
            foot: None
            for foot in FEET
        }

        self.subscribers = []

        for foot in FEET:
            topic = f"/go2/{foot}_foot_contacts"

            sub = rospy.Subscriber(
                topic,
                ContactsState,
                self.callback,
                callback_args=foot,
                queue_size=20,
            )

            self.subscribers.append(sub)

            rospy.loginfo(
                "[CONTACT_RECORDER] subscribing %s",
                topic,
            )

    def callback(self, msg, foot):
        wrench = [0.0] * 6

        # 一个 foot sensor 可能同时存在多个 ContactState。
        # 因此这里对它们的 total_wrench 求和。
        for state in msg.states:
            w = state.total_wrench

            wrench[0] += w.force.x
            wrench[1] += w.force.y
            wrench[2] += w.force.z

            wrench[3] += w.torque.x
            wrench[4] += w.torque.y
            wrench[5] += w.torque.z

        if all(math.isfinite(x) for x in wrench):
            self.values[foot] = wrench
            self.last_wall[foot] = time.monotonic()

    def current(self):
        now = time.monotonic()

        result = {}

        for foot in FEET:
            last = self.last_wall[foot]

            # Swing leg有可能暂时没有新的 contact message。
            # 禁止沿用旧接触力，超时后强制归零。
            if last is None or now - last > self.args.stale_time:
                result[foot] = [0.0] * 6
            else:
                result[foot] = list(self.values[foot])

        return result


def main():
    args = parse_args()

    rospy.init_node(
        "gazebo_contact_wrench_recorder",
        anonymous=False,
    )

    recorder = ContactRecorder(args)

    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)

    columns = ["time"]

    names = ["fx", "fy", "fz", "tx", "ty", "tz"]

    for foot in FEET:
        for name in names:
            columns.append(f"{foot}_{name}")

    for name in names:
        columns.append(f"sum_{name}")

    # ----------------------------------------------------------
    # IMPORTANT:
    # Duration must be measured in Gazebo/ROS simulation time,
    # NOT wall time.
    #
    # During experiment startup Gazebo is paused while the MPC
    # and controller become ready.  Wall time continues running
    # during this pause, while simulation time remains frozen.
    #
    # Therefore we wait until /clock actually begins advancing
    # and only then start the recording-duration clock.
    # ----------------------------------------------------------
    start_ros = None
    watchdog_wall = time.monotonic()

    rate = rospy.Rate(args.rate)

    with output.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(columns)

        while not rospy.is_shutdown():
            now_ros = rospy.Time.now().to_sec()

            # Wait for Gazebo simulation time to start.
            if start_ros is None:
                if now_ros <= 0.0:
                    # Wall-time watchdog only prevents an infinite wait
                    # if Gazebo/controller startup fails completely.
                    if time.monotonic() - watchdog_wall > 180.0:
                        raise RuntimeError(
                            "Gazebo simulation time did not start within 180 s"
                        )

                    time.sleep(0.01)
                    continue

                start_ros = now_ros

            elapsed_ros = now_ros - start_ros

            if elapsed_ros >= args.duration:
                break

            data = recorder.current()

            sums = [0.0] * 6

            row = [
                elapsed_ros
            ]

            for foot in FEET:
                row.extend(data[foot])

                for i in range(6):
                    sums[i] += data[foot][i]

            row.extend(sums)

            writer.writerow(row)

            rate.sleep()

    rospy.loginfo(
        "[CONTACT_RECORDER] wrote %s",
        str(output),
    )


if __name__ == "__main__":
    main()
