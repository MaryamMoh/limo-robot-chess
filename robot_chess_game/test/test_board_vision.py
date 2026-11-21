"""Board vision on synthetic camera images rendered with the same pinhole
model: a top-down view of the board with round piece tops."""
import math

import chess
import cv2
import numpy as np
import pytest

from robot_chess.board_geometry import BoardGeometry
from robot_chess.board_vision import BoardVision, project, square_is_light
from robot_chess.move_inference import infer_move, occupancy

K = np.array([[300.0, 0, 320], [0, 300.0, 240], [0, 0, 1]])
LIGHT, DARK = (120, 180, 205), (40, 75, 115)          # BGR wood colours
WHITE_PIECE, BLACK_PIECE = (225, 235, 238), (25, 22, 22)


def camera_pose(height=0.15, tilt=-0.15):
    """board -> camera optical frame; camera above the board looking down,
    slightly tilted (like the wrist camera)."""
    # optical frame: z forward (down), x right, y down in the image
    rot = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]], float)
    c, s = math.cos(tilt), math.sin(tilt)
    rot = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ rot
    m = np.eye(4)
    m[:3, :3] = rot
    m[:3, 3] = -rot @ np.array([0.0, -0.03, height])
    return m


def render(g, board, cam, noise=6, seed=0):
    img = np.full((480, 640, 3), 90, np.uint8)
    s = g.square_size
    for sq in chess.SQUARES:
        cx, cy = g.square_center_board(sq)
        corners = np.array([[cx - s / 2, cy - s / 2, 0], [cx + s / 2, cy - s / 2, 0],
                            [cx + s / 2, cy + s / 2, 0], [cx - s / 2, cy + s / 2, 0]])
        uv, _ = project(corners, cam, K)
        cv2.fillConvexPoly(img, uv.astype(np.int32), LIGHT if square_is_light(sq) else DARK)
    for sq, piece in board.piece_map().items():
        cx, cy = g.square_center_board(sq)
        uv, z = project(np.array([[cx, cy, 0.012]]), cam, K)
        r = int(K[0, 0] * 0.0065 / z[0])
        cv2.circle(img, tuple(uv[0].astype(int)), r,
                   WHITE_PIECE if piece.color else BLACK_PIECE, -1)
    rng = np.random.RandomState(seed)
    noisy = img.astype(np.int16) + rng.randint(-noise, noise + 1, img.shape)
    return np.clip(noisy, 0, 255).astype(np.uint8)


@pytest.fixture
def setup():
    g = BoardGeometry(square_size=0.02)
    vision = BoardVision(g, sample_height=0.012)
    cam = camera_pose()
    start = chess.Board()
    vision.calibrate(vision.features(render(g, start, cam), cam, K), occupancy(start))
    return g, vision, cam


def test_start_position_read_back(setup):
    g, vision, cam = setup
    occ, conf = vision.classify(vision.features(render(g, chess.Board(), cam, seed=1), cam, K))
    assert occ == occupancy(chess.Board())
    assert min(conf) > 0.2


@pytest.mark.parametrize("moves", [["e4"], ["e4", "d5", "exd5"], ["Nf3", "Nf6", "g3", "g6",
                                                                  "Bg2", "Bg7", "O-O"]])
def test_opponent_moves_recognised_from_images(setup, moves):
    g, vision, cam = setup
    board = chess.Board()
    for san in moves:
        before = board.copy()
        board.push_san(san)
        occ, _ = vision.classify(vision.features(render(g, board, cam, seed=len(board.move_stack)),
                                                 cam, K))
        result = infer_move(before, occ)
        assert result.status == "move" and before.san(result.move) == san


def test_board_out_of_view_detected(setup):
    g, vision, _ = setup
    with pytest.raises(ValueError):
        vision.features(np.zeros((480, 640, 3), np.uint8), camera_pose(height=0.05), K)
