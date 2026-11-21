import math
import xml.etree.ElementTree as ET

import pytest

from robot_chess.odometry_noise import OdometryNoiseModel
from robot_chess.piece_models import piece_sdf
from robot_chess.pieces import TYPES


def _drive_square(model, steps=400):
    """Drive a 2 m x 2 m square in small increments, return final poses."""
    pose = (0.0, 0.0, 0.0)
    model.reset(pose)
    x, y, t = pose
    for _ in range(4):
        for _ in range(steps):
            x += 2.0 / steps * math.cos(t)
            y += 2.0 / steps * math.sin(t)
            model.update((x, y, t))
        for _ in range(steps // 4):
            t = math.atan2(math.sin(t + math.pi / 2 / (steps // 4)),
                           math.cos(t + math.pi / 2 / (steps // 4)))
            model.update((x, y, t))
    return (x, y, t), model.pose


def test_zero_noise_reproduces_ground_truth():
    truth, noisy = _drive_square(OdometryNoiseModel(0, 0, 0, 0))
    assert noisy == pytest.approx(truth, abs=1e-6)


def test_noise_accumulates_but_stays_plausible():
    errors = []
    for seed in range(20):
        truth, noisy = _drive_square(OdometryNoiseModel(seed=seed))
        errors.append(math.hypot(truth[0] - noisy[0], truth[1] - noisy[1]))
    mean = sum(errors) / len(errors)
    assert 0.005 < mean < 0.5


def test_noise_independent_of_update_rate():
    def mean_err(steps):
        errs = []
        for seed in range(30):
            truth, noisy = _drive_square(OdometryNoiseModel(seed=seed), steps)
            errs.append(math.hypot(truth[0] - noisy[0], truth[1] - noisy[1]))
        return sum(errs) / len(errs)
    coarse, fine = mean_err(100), mean_err(800)
    assert 0.4 < fine / coarse < 2.5


def test_backward_motion_is_negative_translation():
    m = OdometryNoiseModel(0, 0, 0, 0)
    m.reset((0.0, 0.0, 0.0))
    pose, (d_trans, d_rot) = m.update((-0.1, 0.0, 0.0))
    assert d_trans == pytest.approx(-0.1)
    assert pose == pytest.approx((-0.1, 0.0, 0.0))


@pytest.mark.parametrize("ptype", TYPES)
def test_piece_sdf_is_valid_xml(ptype):
    root = ET.fromstring(piece_sdf("white_%s_x" % ptype, "white", ptype))
    model = root.find("model")
    assert model.get("name") == "white_%s_x" % ptype
    assert model.find("static").text == "true"
    assert len(model.findall("link/visual")) == 5


def test_turning_in_place_with_simulator_jitter_stays_local():
    """A robot spinning on the spot (with 0.1 mm position jitter, as reported
    by Gazebo) must not get centimetre-level position noise."""
    import random
    jitter = random.Random(1)
    model = OdometryNoiseModel(seed=3)
    model.reset((0.0, 0.0, 0.0))
    yaw = 0.0
    for _ in range(500):                        # 10 s at 50 Hz, 1 rad/s
        yaw += 0.02
        model.update((jitter.gauss(0, 1e-4), jitter.gauss(0, 1e-4),
                      math.atan2(math.sin(yaw), math.cos(yaw))))
    x, y, t = model.pose
    assert math.hypot(x, y) < 0.04
    assert abs(math.atan2(math.sin(t - yaw), math.cos(t - yaw))) < 0.5
