import numpy as np
import pytest

from robot_chess.pieces import BAND_RGB, BODY_RGB, TYPES
from robot_chess.vision import classify_hue, detect_piece


def _bgr(rgb):
    r, g, b = rgb
    return (int(b * 255), int(g * 255), int(r * 255))


def synthetic_view(color, piece_type, noise=8, seed=0):
    """A 640x480 image that resembles what the robot camera sees: grey
    floor/walls, the piece body in the middle and its colour band."""
    rng = np.random.RandomState(seed)
    img = np.full((480, 640, 3), 140, np.uint8)
    img[300:] = 200                                  # light floor
    img[120:470, 250:390] = _bgr(BODY_RGB[color])    # body
    img[200:300, 245:395] = _bgr(BAND_RGB[piece_type])  # band
    noisy = img.astype(np.int16) + rng.randint(-noise, noise + 1, img.shape)
    return np.clip(noisy, 0, 255).astype(np.uint8)


@pytest.mark.parametrize("color", ["white", "black"])
@pytest.mark.parametrize("piece_type", TYPES)
def test_detects_every_piece(color, piece_type):
    det = detect_piece(synthetic_view(color, piece_type))
    assert det.found
    assert det.piece_type == piece_type
    assert det.color == color
    assert det.confidence > 0.5


def test_no_piece_in_view():
    img = np.full((480, 640, 3), 128, np.uint8)
    det = detect_piece(img)
    assert not det.found


def test_hue_classification_borders():
    assert classify_hue(2)[0] == "rook"
    assert classify_hue(178)[0] == "rook"
    assert classify_hue(118)[0] == "knight"
    assert classify_hue(60)[1] == pytest.approx(1.0)
