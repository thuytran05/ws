"""Pure control functions; rear camera, ROS base x forward, z yaw left."""
import math


def commands(x, z, desired=1.5, distance_deadband=0.1,
             angle_deadband=0.05, kp_distance=0.20, kp_yaw=0.45,
             max_speed=0.10, max_yaw=0.20):
    error = z - desired
    # Subtract the deadband to avoid a jump at its boundary.
    distance = math.copysign(max(0.0, abs(error) - distance_deadband), error)
    angle = math.atan2(x, z)
    turn = math.copysign(max(0.0, abs(angle) - angle_deadband), angle)
    vx = max(-max_speed, min(max_speed, -kp_distance * distance))
    # Progressive yaw: gentle near center, stronger near image edges.
    wz = max(-max_yaw, min(max_yaw, -kp_yaw * turn * abs(turn) / 0.30))
    # Turn first when target is well away from center.
    vx *= max(0.0, 1.0 - abs(angle) / 0.40)
    return vx, wz


def slew(current, desired, rate, dt):
    return current + max(-rate * dt, min(rate * dt, desired - current))
