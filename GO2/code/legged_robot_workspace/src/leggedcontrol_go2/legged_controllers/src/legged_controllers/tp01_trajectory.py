import math
from collections import namedtuple


ReferencePoint = namedtuple(
    "ReferencePoint", ("time", "pose", "velocity", "acceleration"))

def _wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _quintic_scale(tau):
    scale = 10.0 * tau ** 3 - 15.0 * tau ** 4 + 6.0 * tau ** 5
    first = 30.0 * tau ** 2 - 60.0 * tau ** 3 + 30.0 * tau ** 4
    second = 60.0 * tau - 180.0 * tau ** 2 + 120.0 * tau ** 3
    return scale, first, second


def _cruise_scale(elapsed, duration, accel_duration):
    """Smooth acceleration, constant cruise, smooth braking; return time derivatives."""
    if duration <= 2.0 * accel_duration or accel_duration <= 0.0:
        raise ValueError("acceleration duration must fit twice inside trajectory duration")
    peak_rate = 1.0 / (duration - accel_duration)
    if elapsed < accel_duration:
        u = elapsed / accel_duration
        return (peak_rate * accel_duration * (u ** 3 - 0.5 * u ** 4),
                peak_rate * (3.0 * u ** 2 - 2.0 * u ** 3),
                peak_rate * (6.0 * u - 6.0 * u ** 2) / accel_duration)
    if elapsed <= duration - accel_duration:
        return (peak_rate * (elapsed - 0.5 * accel_duration), peak_rate, 0.0)
    u = (duration - elapsed) / accel_duration
    return (1.0 - peak_rate * accel_duration * (u ** 3 - 0.5 * u ** 4),
            peak_rate * (3.0 * u ** 2 - 2.0 * u ** 3),
            -peak_rate * (6.0 * u - 6.0 * u ** 2) / accel_duration)


def make_relative_goal(start_pose, dx, dy, dyaw, com_height):
    if len(start_pose) != 6:
        raise ValueError("start_pose must contain six values")
    return (
        float(start_pose[0]) + float(dx),
        float(start_pose[1]) + float(dy),
        float(com_height),
        float(start_pose[3]) + _wrap_angle(float(dyaw)),
        0.0,
        0.0,
    )


def reference_state_values(point, template_state, joint_state=None):
    if len(template_state) < 12:
        raise ValueError("template_state must contain a centroidal base state")
    values = [0.0] * len(template_state)
    # The native cmd_vel conversion uses the first three normalized momentum
    # targets for world-frame linear velocity. Angular momentum remains zero
    # because these experiments hold body heading fixed.
    values[0:3] = point.velocity[0:3]
    values[6:12] = point.pose
    if joint_state is None:
        values[12:] = template_state[12:]
    else:
        if len(joint_state) != len(template_state) - 12:
            raise ValueError("joint_state size must match the target state")
        values[12:] = tuple(float(value) for value in joint_state)
    return tuple(values)


def build_target_payload(reference, template_state, input_size, joint_state=None):
    if input_size < 0:
        raise ValueError("input_size must be nonnegative")
    return {
        "times": tuple(point.time for point in reference),
        "states": tuple(
            reference_state_values(point, template_state, joint_state)
            for point in reference
        ),
        "inputs": tuple(
            (0.0,) * input_size for _point in reference
        ),
    }


def planner_rows(reference, start_elapsed_s):
    if not reference:
        return []
    first_time = reference[0].time
    rows = []
    for point in reference:
        pose = point.pose
        velocity = point.velocity
        acceleration = point.acceleration
        rows.append({
            "trajectory_time_s": point.time,
            "campaign_elapsed_s": (
                float(start_elapsed_s) + point.time - first_time),
            "x_m": pose[0],
            "y_m": pose[1],
            "z_m": pose[2],
            "yaw_rad": pose[3],
            "pitch_rad": pose[4],
            "roll_rad": pose[5],
            "vx_mps": velocity[0],
            "vy_mps": velocity[1],
            "vz_mps": velocity[2],
            "yaw_rate_radps": velocity[3],
            "pitch_rate_radps": velocity[4],
            "roll_rate_radps": velocity[5],
            "ax_mps2": acceleration[0],
            "ay_mps2": acceleration[1],
            "az_mps2": acceleration[2],
            "yaw_accel_radps2": acceleration[3],
            "pitch_accel_radps2": acceleration[4],
            "roll_accel_radps2": acceleration[5],
        })
    return rows


def build_quintic_reference(
        start_pose, goal_pose, start_time, duration, rate_hz):
    if duration <= 0.0:
        raise ValueError("duration must be positive")
    if rate_hz <= 0.0:
        raise ValueError("rate_hz must be positive")
    if len(start_pose) != 6 or len(goal_pose) != 6:
        raise ValueError("start_pose and goal_pose must contain six values")

    start_pose = tuple(float(value) for value in start_pose)
    goal_pose = tuple(float(value) for value in goal_pose)
    delta = [goal - start for start, goal in zip(start_pose, goal_pose)]
    delta[3] = _wrap_angle(goal_pose[3] - start_pose[3])
    step_count = int(round(float(duration) * float(rate_hz)))
    reference = []
    for index in range(step_count + 1):
        tau = index / float(step_count)
        scale, first, second = _quintic_scale(tau)
        pose = tuple(
            start_pose[axis] + scale * delta[axis]
            for axis in range(6)
        )
        velocity = tuple(
            first * delta[axis] / duration for axis in range(6)
        )
        acceleration = tuple(
            second * delta[axis] / (duration * duration)
            for axis in range(6)
        )
        reference.append(ReferencePoint(
            time=float(start_time) + tau * float(duration),
            pose=pose,
            velocity=velocity,
            acceleration=acceleration,
        ))
    return reference


def build_cruise_reference(start_pose, goal_pose, start_time, duration,
                           rate_hz, accel_duration):
    if duration <= 2.0 * accel_duration or accel_duration <= 0.0 or rate_hz <= 0.0:
        raise ValueError("invalid cruise reference duration or rate")
    if len(start_pose) != 6 or len(goal_pose) != 6:
        raise ValueError("start and goal poses must contain six values")
    start_pose = tuple(float(value) for value in start_pose)
    delta = [float(goal) - start for start, goal in zip(start_pose, goal_pose)]
    delta[3] = _wrap_angle(float(goal_pose[3]) - start_pose[3])
    step_count = int(round(float(duration) * float(rate_hz)))
    reference = []
    for index in range(step_count + 1):
        elapsed = index * float(duration) / step_count
        progress, rate, accel = _cruise_scale(elapsed, duration, accel_duration)
        reference.append(ReferencePoint(
            time=float(start_time) + elapsed,
            pose=tuple(start_pose[axis] + progress * delta[axis] for axis in range(6)),
            velocity=tuple(rate * value for value in delta),
            acceleration=tuple(accel * value for value in delta),
        ))
    return reference


def build_single_bend_reference(start_pose, forward_length, lateral_amplitude,
                                start_time, duration, rate_hz, accel_duration=None):
    """One gentle lateral bend, returning to the centreline at fixed heading."""
    if len(start_pose) != 6:
        raise ValueError("start_pose must contain six values")
    if forward_length <= 0.0 or lateral_amplitude <= 0.0:
        raise ValueError("single-bend dimensions must be positive")
    if duration <= 0.0 or rate_hz <= 0.0:
        raise ValueError("duration and rate must be positive")
    start_pose = tuple(float(value) for value in start_pose)
    yaw0 = start_pose[3]
    cy, sy = math.cos(yaw0), math.sin(yaw0)
    step_count = int(round(float(duration) * float(rate_hz)))
    reference = []
    for index in range(step_count + 1):
        tau = index / float(step_count)
        if accel_duration is None:
            progress, progress_rate, progress_accel = _quintic_scale(tau)
            progress_rate /= duration
            progress_accel /= duration * duration
        else:
            progress, progress_rate, progress_accel = _cruise_scale(
                tau * duration, duration, accel_duration)
        angle = math.pi * progress
        shape = math.sin(angle) ** 2
        shape_d = math.pi * math.sin(2.0 * angle)
        shape_dd = 2.0 * math.pi ** 2 * math.cos(2.0 * angle)
        local_x = float(forward_length) * progress
        local_y = float(lateral_amplitude) * shape
        dx_ds = float(forward_length)
        dy_ds = float(lateral_amplitude) * shape_d
        d2y_ds2 = float(lateral_amplitude) * shape_dd
        local_vx = dx_ds * progress_rate
        local_vy = dy_ds * progress_rate
        local_ax = dx_ds * progress_accel
        local_ay = d2y_ds2 * progress_rate ** 2 + dy_ds * progress_accel
        x = start_pose[0] + cy * local_x - sy * local_y
        y = start_pose[1] + sy * local_x + cy * local_y
        vx = cy * local_vx - sy * local_vy
        vy = sy * local_vx + cy * local_vy
        ax = cy * local_ax - sy * local_ay
        ay = sy * local_ax + cy * local_ay
        reference.append(ReferencePoint(
            time=float(start_time) + tau * float(duration),
            pose=(x, y, start_pose[2], yaw0, 0.0, 0.0),
            velocity=(vx, vy, 0.0, 0.0, 0.0, 0.0),
            acceleration=(ax, ay, 0.0, 0.0, 0.0, 0.0),
        ))
    return reference


def append_terminal_hold(reference, hold_duration):
    if not reference or hold_duration < 0.0:
        raise ValueError("reference must be nonempty and hold duration nonnegative")
    if hold_duration == 0.0:
        return list(reference)
    final = reference[-1]
    return list(reference) + [ReferencePoint(
        time=final.time + float(hold_duration), pose=final.pose,
        velocity=(0.0,) * 6, acceleration=(0.0,) * 6)]
