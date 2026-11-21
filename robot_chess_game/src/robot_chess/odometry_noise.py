"""Noisy odometry motion model.

Gazebo's drive plugin reports a perfect odometry.  To work with a realistic
robot we corrupt it with an odometry motion model (Thrun, Burgard & Fox,
"Probabilistic Robotics", ch. 5.4).  Every motion increment between two
ground-truth poses is expressed in the robot frame as

    trans = forward displacement   (signed, projection on the old heading)
    rot   = change of heading

which is exactly what a differential drive's wheel encoders measure.  Both
parts get zero-mean Gaussian noise and the noisy increment is integrated
(midpoint rule) into the reported pose:

    var(rot)   = a1 * |rot|   + a2 * |trans|
    var(trans) = a3 * |trans| + a4 * |rot|

Two deliberate differences to the textbook (rot1, trans, rot2) form:
  * the variances are proportional to the *absolute* increments (not their
    squares), so the error grows like a random walk with travelled distance
    and rotation and does not depend on the update rate;
  * no "rotation towards the direction of motion" is computed: for a robot
    turning in place that angle is undefined and the tiny translational
    jitter of the simulator would turn it into huge spurious noise.
"""
import math
import random


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class OdometryNoiseModel(object):

    def __init__(self, alpha1=0.002, alpha2=0.001, alpha3=0.002, alpha4=0.00001,
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

        trans = (x1 - x0) * math.cos(t0) + (y1 - y0) * math.sin(t0)
        rot = _wrap(t1 - t0)

        a1, a2, a3, a4 = self.alpha
        n_rot = rot + self._sample(a1 * abs(rot) + a2 * abs(trans))
        n_trans = trans + self._sample(a3 * abs(trans) + a4 * abs(rot))

        x, y, t = self.pose
        mid = t + n_rot / 2.0
        x += n_trans * math.cos(mid)
        y += n_trans * math.sin(mid)
        t = _wrap(t + n_rot)
        self.pose = (x, y, t)
        return self.pose, (n_trans, n_rot)
