"""Noisy odometry motion model.

Gazebo's drive plugin reports a perfect odometry.  To work with a realistic
robot we corrupt it with the odometry motion model of Thrun, Burgard & Fox
("Probabilistic Robotics", ch. 5.4): every motion increment is split into
rotation-1, translation, rotation-2 and each part gets zero-mean Gaussian
noise.

The book uses variances proportional to the *squared* increments, which
makes the accumulated error depend on the update rate (at 50 Hz the
increments are tiny and the noise almost vanishes).  We use variances
proportional to the *absolute* increments instead, so the error grows like a
random walk with travelled distance / rotation, independent of the rate:

    var(rot1)  = a1 * |rot1|  + a2 * |trans|
    var(trans) = a3 * |trans| + a4 * (|rot1| + |rot2|)
    var(rot2)  = a1 * |rot2|  + a2 * |trans|
"""
import math
import random


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdometryNoiseModel(object):

    def __init__(self, alpha1=0.002, alpha2=0.001, alpha3=0.002, alpha4=0.001,
                 seed=None):
        self.alpha = (float(alpha1), float(alpha2), float(alpha3), float(alpha4))
        self.rng = random.Random(seed)
        self.prev_true = None
        self.pose = None          # noisy (x, y, yaw)

    def reset(self, true_pose, noisy_pose=None):
        """Start tracking.  `noisy_pose` is the initial reported pose, e.g.
        (0, 0, 0) when odometry starts at the origin of the odom frame while
        the ground truth is expressed in the world frame."""
        self.prev_true = tuple(true_pose)
        self.pose = tuple(noisy_pose if noisy_pose is not None else true_pose)

    def _sample(self, variance):
        return self.rng.gauss(0.0, math.sqrt(max(variance, 0.0)))

    def update(self, true_pose):
        """Feed the next ground-truth pose; returns the new noisy pose and the
        noisy increment (d_trans, d_rot) in the robot frame."""
        if self.prev_true is None:
            self.reset(true_pose)
            return self.pose, (0.0, 0.0)
        x0, y0, t0 = self.prev_true
        x1, y1, t1 = true_pose
        self.prev_true = tuple(true_pose)

        trans = math.hypot(x1 - x0, y1 - y0)
        if trans < 1e-6:
            rot1 = 0.0
        else:
            rot1 = _wrap(math.atan2(y1 - y0, x1 - x0) - t0)
            # Driving backwards: express as a negative translation.
            if abs(rot1) > math.pi / 2.0:
                rot1 = _wrap(rot1 - math.pi)
                trans = -trans
        rot2 = _wrap(t1 - t0 - rot1)

        a1, a2, a3, a4 = self.alpha
        n_rot1 = rot1 - self._sample(a1 * abs(rot1) + a2 * abs(trans))
        n_trans = trans - self._sample(a3 * abs(trans) + a4 * (abs(rot1) + abs(rot2)))
        n_rot2 = rot2 - self._sample(a1 * abs(rot2) + a2 * abs(trans))

        x, y, t = self.pose
        x += n_trans * math.cos(t + n_rot1)
        y += n_trans * math.sin(t + n_rot1)
        t = _wrap(t + n_rot1 + n_rot2)
        self.pose = (x, y, t)
        return self.pose, (n_trans, n_rot1 + n_rot2)
