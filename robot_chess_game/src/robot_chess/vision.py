"""Camera based piece identification.

Every piece wears a saturated colour band at camera height; the band hue
identifies the piece type and the (unsaturated) body colour just below the
band identifies the side.  The robot stops ~0.25 m in front of the piece,
so the piece fills the centre of the image.

Pipeline (OpenCV):
  1. keep the central vertical strip of the image (the piece is in front);
  2. HSV threshold on saturation/value -> mask of "band-like" pixels;
  3. morphological opening, take the largest connected blob;
  4. circular mean of the blob hue -> nearest type hue;
  5. median brightness of low-saturation pixels right below the blob
     -> white or black body.
"""
import math

import cv2
import numpy as np

from robot_chess.pieces import BAND_HUE_CV

MIN_SATURATION = 110
MIN_VALUE = 60
BODY_MAX_SATURATION = 90
WHITE_BODY_MIN_VALUE = 128


class Detection(object):
    __slots__ = ("found", "color", "piece_type", "confidence", "bbox", "message")

    def __init__(self, found=False, color="", piece_type="", confidence=0.0,
                 bbox=None, message=""):
        self.found = found
        self.color = color
        self.piece_type = piece_type
        self.confidence = confidence
        self.bbox = bbox
        self.message = message

    def __repr__(self):
        return "Detection(found=%s, %s %s, conf=%.2f)" % (
            self.found, self.color, self.piece_type, self.confidence)


def _hue_distance(h1, h2):
    """Distance on the OpenCV hue circle (0..179)."""
    d = abs(h1 - h2) % 180
    return min(d, 180 - d)


def _circular_mean_hue(hues):
    angles = hues.astype(np.float64) * (2.0 * math.pi / 180.0)
    mean = math.atan2(np.sin(angles).mean(), np.cos(angles).mean())
    return (mean * 180.0 / (2.0 * math.pi)) % 180.0


def classify_hue(hue):
    """Nearest piece type for an OpenCV hue, plus a 0..1 confidence."""
    best = min(BAND_HUE_CV, key=lambda t: _hue_distance(hue, BAND_HUE_CV[t]))
    dist = _hue_distance(hue, BAND_HUE_CV[best])
    # Neighbouring classes are 30 apart -> 15 is the decision border.
    return best, max(0.0, 1.0 - dist / 15.0)


def detect_piece(bgr, strip_fraction=0.5, min_area_fraction=0.004):
    """Identify the piece in the centre of a BGR image."""
    if bgr is None or bgr.size == 0:
        return Detection(message="empty image")
    h, w = bgr.shape[:2]
    x0 = int(w * (1.0 - strip_fraction) / 2.0)
    x1 = w - x0
    strip = bgr[:, x0:x1]
    hsv = cv2.cvtColor(strip, cv2.COLOR_BGR2HSV)

    mask = ((hsv[:, :, 1] >= MIN_SATURATION) &
            (hsv[:, :, 2] >= MIN_VALUE)).astype(np.uint8) * 255
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    if count <= 1:
        return Detection(message="no coloured band visible")
    largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    area = stats[largest, cv2.CC_STAT_AREA]
    if area < min_area_fraction * h * w:
        return Detection(message="band too small (%d px)" % area)

    bx, by, bw, bh = (stats[largest, cv2.CC_STAT_LEFT], stats[largest, cv2.CC_STAT_TOP],
                      stats[largest, cv2.CC_STAT_WIDTH], stats[largest, cv2.CC_STAT_HEIGHT])
    hues = hsv[:, :, 0][labels == largest]
    piece_type, hue_conf = classify_hue(_circular_mean_hue(hues))

    # Body: the rows right under the band, same columns, inner 60 % width.
    cx0 = bx + int(0.2 * bw)
    cx1 = max(cx0 + 1, bx + int(0.8 * bw))
    ry0 = min(h - 1, by + bh + 2)
    ry1 = min(h, ry0 + max(4, bh // 2))
    body = hsv[ry0:ry1, cx0:cx1].reshape(-1, 3)
    if body.shape[0] < 4:
        # Band touches the image bottom: look above it instead.
        ry1 = max(0, by - 2)
        ry0 = max(0, ry1 - max(4, bh // 2))
        body = hsv[ry0:ry1, cx0:cx1].reshape(-1, 3)
    body = body[body[:, 1] <= BODY_MAX_SATURATION]
    if body.shape[0] < 4:
        return Detection(found=True, piece_type=piece_type, confidence=hue_conf * 0.5,
                         bbox=(bx + x0, by, bw, bh),
                         message="band found but body colour not visible")
    value = float(np.median(body[:, 2]))
    color = "white" if value >= WHITE_BODY_MIN_VALUE else "black"
    color_conf = min(1.0, abs(value - WHITE_BODY_MIN_VALUE) / 64.0)

    return Detection(found=True, color=color, piece_type=piece_type,
                     confidence=float(hue_conf * (0.5 + 0.5 * color_conf)),
                     bbox=(bx + x0, by, bw, bh),
                     message="%s %s (hue conf %.2f, body V=%.0f)" % (
                         color, piece_type, hue_conf, value))


def draw_detection(bgr, detection):
    """Annotated copy of the image for debugging in RViz / rqt_image_view."""
    out = bgr.copy()
    if detection.bbox is not None:
        x, y, w, h = detection.bbox
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 255), 2)
    text = detection.message if not detection.found else "%s %s %.2f" % (
        detection.color, detection.piece_type, detection.confidence)
    cv2.putText(out, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255), 2, cv2.LINE_AA)
    return out
