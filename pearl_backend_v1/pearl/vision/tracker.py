"""
Cheap classical tracker for frame-to-frame target centering.

The VLM is identity verification, not tracking: it runs every ~1.5 s. Between
verifications this tracker follows the confirmed bbox at ~4 Hz using grayscale
template matching in a local search window on a 160-px-wide image. That is a
few hundred microseconds on a Pi 5.

If opencv-contrib is installed, cv2.legacy.TrackerKCF is used instead (better
with scale change). TODO: evaluate which is more robust for people vs objects.
"""
from __future__ import annotations

import time
from typing import Optional

import cv2
import numpy as np

from .. import config
from ..logging_setup import get_logger

log = get_logger(__name__)


def _to_work(frame: np.ndarray) -> tuple[np.ndarray, float]:
    """Grayscale, downscaled to VISION_TRACKER_WORK_WIDTH. Returns (img, scale)."""
    h, w = frame.shape[:2]
    scale = config.VISION_TRACKER_WORK_WIDTH / float(w)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    small = cv2.resize(gray, (config.VISION_TRACKER_WORK_WIDTH, max(1, int(h * scale))), interpolation=cv2.INTER_AREA)
    return small, scale


class TemplateTracker:
    def __init__(self) -> None:
        self.active = False
        self._tpl: Optional[np.ndarray] = None
        self._box: Optional[tuple[int, int, int, int]] = None   # x, y, w, h in work coords
        self._misses = 0
        self._kcf = None
        self.last_bbox_norm: Optional[list[float]] = None
        self.last_ts = 0.0

    def init(self, frame: np.ndarray, bbox_norm: list[float]) -> None:
        work, _ = _to_work(frame)
        H, W = work.shape[:2]
        x0, y0 = int(bbox_norm[0] * W), int(bbox_norm[1] * H)
        x1, y1 = int(bbox_norm[2] * W), int(bbox_norm[3] * H)
        w, h = max(4, x1 - x0), max(4, y1 - y0)
        x0, y0 = max(0, min(W - w, x0)), max(0, min(H - h, y0))
        self._box = (x0, y0, w, h)
        self._tpl = work[y0:y0 + h, x0:x0 + w].copy()
        self._misses = 0
        self.active = True
        self.last_bbox_norm = list(bbox_norm)
        self.last_ts = time.time()
        self._kcf = None
        if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerKCF_create"):
            try:
                self._kcf = cv2.legacy.TrackerKCF_create()
                self._kcf.init(cv2.cvtColor(work, cv2.COLOR_GRAY2BGR), self._box)
            except cv2.error:
                self._kcf = None

    def drop(self) -> None:
        self.active = False
        self._tpl = None
        self._kcf = None
        self.last_bbox_norm = None

    def update(self, frame: np.ndarray) -> Optional[list[float]]:
        """Returns normalized bbox or None (lost this frame)."""
        if not self.active or self._box is None:
            return None
        work, _ = _to_work(frame)
        H, W = work.shape[:2]
        x, y, w, h = self._box
        found: Optional[tuple[int, int]] = None

        if self._kcf is not None:
            ok, box = self._kcf.update(cv2.cvtColor(work, cv2.COLOR_GRAY2BGR))
            if ok:
                found = (int(box[0]), int(box[1]))
        elif self._tpl is not None:
            # search window = box grown by 100% each side
            sx0, sy0 = max(0, x - w), max(0, y - h)
            sx1, sy1 = min(W, x + 2 * w), min(H, y + 2 * h)
            win = work[sy0:sy1, sx0:sx1]
            if win.shape[0] >= h and win.shape[1] >= w:
                res = cv2.matchTemplate(win, self._tpl, cv2.TM_CCOEFF_NORMED)
                _, score, _, loc = cv2.minMaxLoc(res)
                if score >= config.VISION_TRACKER_SCORE_MIN:
                    found = (sx0 + loc[0], sy0 + loc[1])

        if found is None:
            self._misses += 1
            if self._misses >= config.VISION_TRACKER_MAX_MISSES:
                log.debug("tracker dropped after %d misses", self._misses)
                self.drop()
            return None

        self._misses = 0
        self._box = (found[0], found[1], w, h)
        self.last_bbox_norm = [found[0] / W, found[1] / H, (found[0] + w) / W, (found[1] + h) / H]
        self.last_ts = time.time()
        return self.last_bbox_norm
