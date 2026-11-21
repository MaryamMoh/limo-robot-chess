"""Board occupancy from a camera image.

1. Projection.  The camera pose relative to the board is known (wrist
   camera: forward kinematics of the arm -> TF; fixed camera: static TF),
   so every square centre can be projected into the image with the pinhole
   model  u = fx * X / Z + cx,  v = fy * Y / Z + cy.  The point used is
   `sample_height` above the board: the top of a piece, so tall pieces are
   sampled where they really appear (parallax).
2. Features.  Mean colour (CIE Lab, robust to brightness changes) of a
   small window around every projected point.
3. Classification.  Nearest centroid among four classes: empty light square,
   empty dark square, white piece, black piece.  The centroids are learnt
   from one image of the start position - its occupancy is known - so the
   classifier adapts to the real board, pieces and lighting by itself.
   The confidence of a square is the relative margin between the nearest
   and the second-nearest class of a different occupancy.
"""
import chess
import cv2
import numpy as np

from robot_chess.move_inference import BLACK, EMPTY, WHITE

CLASSES = ("light", "dark", WHITE, BLACK)
CLASS_OCCUPANCY = {"light": EMPTY, "dark": EMPTY, WHITE: WHITE, BLACK: BLACK}


def square_is_light(square):
    return (chess.square_file(square) + chess.square_rank(square)) % 2 == 1


def project(points_board, cam_from_board, K):
    """Project Nx3 board-frame points; returns (Nx2 pixels, N depths)."""
    pts = np.hstack([points_board, np.ones((len(points_board), 1))])
    cam = (cam_from_board @ pts.T).T[:, :3]
    z = cam[:, 2]
    uv = np.stack([K[0, 0] * cam[:, 0] / z + K[0, 2],
                   K[1, 1] * cam[:, 1] / z + K[1, 2]], axis=1)
    return uv, z


class BoardVision(object):

    def __init__(self, geometry, sample_height=0.012, window_fraction=0.25):
        self.g = geometry
        self.sample_height = float(sample_height)
        self.window_fraction = float(window_fraction)
        self.centroids = None

    def sample_points(self):
        pts = [list(self.g.square_center_board(sq)) + [self.sample_height]
               for sq in chess.SQUARES]
        return np.array(pts)

    def features(self, image_bgr, cam_from_board, K):
        """Nx3 mean Lab colour per square; raises ValueError if the board is
        not completely in view."""
        uv, z = project(self.sample_points(), cam_from_board, K)
        h, w = image_bgr.shape[:2]
        if np.any(z <= 0) or np.any(uv < 0) or np.any(uv[:, 0] >= w) or np.any(uv[:, 1] >= h):
            raise ValueError("board not completely in the camera view")
        lab = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2LAB).astype(np.float32)
        feats = np.zeros((64, 3), np.float32)
        for sq in chess.SQUARES:
            r = max(2, int(K[0, 0] * self.window_fraction * self.g.square_size / z[sq]))
            u, v = int(round(uv[sq, 0])), int(round(uv[sq, 1]))
            patch = lab[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
            feats[sq] = patch.reshape(-1, 3).mean(axis=0)
        return feats

    # ---------------------------------------------------------- learning
    def calibrate(self, feats, known_occupancy):
        """Learn the class centroids from an image with known occupancy
        (normally the start position)."""
        groups = {c: [] for c in CLASSES}
        for sq in chess.SQUARES:
            occ = known_occupancy[sq]
            if occ == EMPTY:
                groups["light" if square_is_light(sq) else "dark"].append(feats[sq])
            else:
                groups[occ].append(feats[sq])
        missing = [c for c, v in groups.items() if not v]
        if missing:
            raise ValueError("cannot calibrate, no example of %s" % missing)
        self.centroids = {c: np.mean(v, axis=0) for c, v in groups.items()}

    def classify(self, feats):
        """(occupancy string, list of 64 confidences in 0..1)."""
        if self.centroids is None:
            raise RuntimeError("vision not calibrated")
        occ, conf = [], []
        for sq in chess.SQUARES:
            dist = {c: float(np.linalg.norm(feats[sq] - m)) for c, m in self.centroids.items()}
            best = min(dist, key=dist.get)
            other = min(d for c, d in dist.items()
                        if CLASS_OCCUPANCY[c] != CLASS_OCCUPANCY[best])
            occ.append(CLASS_OCCUPANCY[best])
            conf.append((other - dist[best]) / (other + dist[best] + 1e-6))
        return "".join(occ), conf

    def draw(self, image_bgr, cam_from_board, K, occupancy=None):
        """Debug image: projected square centres and the classification."""
        out = image_bgr.copy()
        uv, _ = project(self.sample_points(), cam_from_board, K)
        colors = {EMPTY: (0, 200, 0), WHITE: (255, 255, 255), BLACK: (0, 0, 255)}
        for sq in chess.SQUARES:
            u, v = int(uv[sq, 0]), int(uv[sq, 1])
            c = colors[occupancy[sq]] if occupancy else (0, 255, 255)
            cv2.circle(out, (u, v), 4, c, -1)
            cv2.circle(out, (u, v), 5, (0, 0, 0), 1)
        return out
