"""Forward and inverse kinematics of the myCobot 280 mounted on the LIMO.

The joint table below is the kinematic chain of AgileX's LIMO + myCobot
model (limo_cobot_sim, limo_base.urdf) - the same numbers are used in
robot_chess_description/urdf/limo_mycobot.urdf.xacro, so the Python model
and the simulated / real arm agree.

Frames
    base_footprint   robot frame on the floor (x forward, z up)
    joint1           arm base, at base_link = base_footprint + (0, 0, 0.15),
                     shifted by the mount offset (-0.01, 0, 0)
    joint6_flange    tool flange; its +z axis is the tool axis
    TCP              tool centre point = flange + tcp_offset * z_flange
                     (the point between the gripper finger tips)

Inverse kinematics: damped least squares on 5 constraints - TCP position
(3) and tool axis pointing straight down (2); the rotation about the tool
axis is left free (chess pieces are round when grasped from above) unless a
yaw is requested (6th constraint: direction of the flange x axis), which is
used to point the wrist camera at the board.
"""
import math

import numpy as np

# (xyz, rpy) of every revolute joint's origin, joint axis is z.
JOINTS = [
    ((0.0, 0.0, 0.13156), (0.0, 0.0, 0.0)),            # joint2_to_joint1
    ((0.0, 0.0, 0.0), (0.0, 1.5708, -1.5708)),         # joint3_to_joint2
    ((-0.1104, 0.0, 0.0), (0.0, 0.0, 0.0)),            # joint4_to_joint3
    ((-0.096, 0.0, 0.06462), (0.0, 0.0, -1.5708)),     # joint5_to_joint4
    ((0.0, -0.07318, 0.0), (1.5708, -1.5708, 0.0)),    # joint6_to_joint5
    ((0.0, 0.0456, 0.0), (-1.5708, 0.0, 0.0)),         # joint6output_to_joint6
]
JOINT_NAMES = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
               "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
# myCobot 280 joint range is +-165 deg (joint 6: +-175 deg)
JOINT_LIMITS = [math.radians(165)] * 5 + [math.radians(175)]

ARM_MOUNT = (-0.01, 0.0, 0.15)      # joint1 origin in base_footprint
DOWN = np.array([0.0, 0.0, -1.0])


def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(xyz, rpy):
    m = np.eye(4)
    m[:3, :3] = rpy_matrix(*rpy)
    m[:3, 3] = xyz
    return m


def rot_z(q):
    m = np.eye(4)
    c, s = math.cos(q), math.sin(q)
    m[:2, :2] = [[c, -s], [s, c]]
    return m


_ORIGINS = [transform(xyz, rpy) for xyz, rpy in JOINTS]


class MyCobotKinematics(object):

    def __init__(self, tcp_offset=0.06, mount=ARM_MOUNT):
        self.tcp_offset = float(tcp_offset)
        self.base = transform(mount, (0.0, 0.0, 0.0))

    # ------------------------------------------------------------- forward
    def flange(self, q):
        """4x4 pose of joint6_flange in base_footprint."""
        m = self.base.copy()
        for origin, qi in zip(_ORIGINS, q):
            m = m @ origin @ rot_z(qi)
        return m

    def tcp(self, q):
        """(position, tool_axis) of the tool centre point in base_footprint."""
        f = self.flange(q)
        axis = f[:3, 2]
        return f[:3, 3] + self.tcp_offset * axis, axis

    def frame_on_flange(self, q, offset):
        """Pose of a frame rigidly attached to the flange (4x4 offset)."""
        return self.flange(q) @ offset

    # ------------------------------------------------------------- inverse
    def _error(self, q, target, yaw=None):
        f = self.flange(q)
        z = f[:3, 2]
        p = f[:3, 3] + self.tcp_offset * z
        e = [target - p, np.cross(z, DOWN)[:2]]
        if yaw is not None:
            x = f[:3, 0]
            e.append([math.cos(yaw) * x[1] - math.sin(yaw) * x[0]])
        return np.concatenate(e)

    def ik_down(self, target, seed=None, yaw=None, tolerance=5e-4, iterations=300):
        """Joint angles that put the TCP at `target` (base_footprint) with
        the tool pointing straight down, or None if unreachable.  `yaw`
        (optional): direction of the flange x axis in the horizontal plane.

        Several seeds are tried; the solution closest to `seed` wins."""
        target = np.asarray(target, float)
        seeds = [seed] if seed is not None else []
        seeds += [[0.0, -0.6, -1.2, 0.3, 0.0, 0.0],
                  [math.atan2(target[1], target[0]), -0.3, -1.6, 0.4, 0.0, 0.0],
                  [math.atan2(target[1], target[0]), -1.0, -0.6, 0.0, 0.0, 0.0]]
        best = None
        for s in seeds:
            q = self._solve(target, np.array(s, float), yaw, tolerance, iterations)
            if q is None:
                continue
            ref = np.array(seed if seed is not None else seeds[0], float)
            cost = float(np.abs(q - ref).sum())
            if best is None or cost < best[0]:
                best = (cost, q)
            if seed is not None:
                break
        return None if best is None else [float(v) for v in best[1]]

    def _solve(self, target, q, yaw, tolerance, iterations):
        limits = np.array(JOINT_LIMITS)
        damping = 0.03
        h = 1e-6
        n = 5 if yaw is None else 6
        for _ in range(iterations):
            e = self._error(q, target, yaw)
            if np.linalg.norm(e[:3]) < tolerance and np.linalg.norm(e[3:]) < 2e-3:
                if yaw is None or self._yaw_ok(q, yaw):
                    return q
            jac = np.zeros((n, 6))
            for i in range(6):
                dq = q.copy()
                dq[i] += h
                jac[:, i] = -(self._error(dq, target, yaw) - e) / h
            step = jac.T @ np.linalg.solve(jac @ jac.T + damping ** 2 * np.eye(n), e)
            norm = np.linalg.norm(step)
            if norm > 0.3:
                step *= 0.3 / norm
            q = np.clip(q + step, -limits, limits)
        return None

    def _yaw_ok(self, q, yaw):
        x = self.flange(q)[:3, 0]
        return math.cos(yaw) * x[0] + math.sin(yaw) * x[1] > 0.99

    def reachable(self, target, seed=None):
        return self.ik_down(target, seed) is not None


def interpolate_line(start, end, step=0.01):
    """Points on the straight segment start -> end, about `step` apart."""
    start, end = np.asarray(start, float), np.asarray(end, float)
    n = max(1, int(math.ceil(np.linalg.norm(end - start) / step)))
    return [start + (end - start) * (i / float(n)) for i in range(1, n + 1)]
