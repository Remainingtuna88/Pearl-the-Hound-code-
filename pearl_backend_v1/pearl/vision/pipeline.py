"""
VisionPipeline — one thread, two cadences.

  every VISION_TRACK_INTERVAL_S (0.25 s): if a tracker is active, step it on
      the latest frame (grayscale, 160 px wide — cheap) and publish the bbox as
      source="tracker".
  every VISION_VLM_INTERVAL_S (1.5 s):   encode a 320-px JPEG and ask the VLM
      whether the target is visible. A confirmed match (re)initialises the
      tracker; a confident "no" drops it.

Frames are read by reference from RobotInterface.latest_frame(); the only copy
is the JPEG encode of the downscaled image. Nothing runs when no target is set.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Optional, Protocol

from .. import config
from ..logging_setup import get_logger, log_event
from ..robot.interface import RobotInterface
from ..state import SharedState, VisionResult
from .tracker import TemplateTracker
from .vlm_client import bearing_from_bbox

log = get_logger(__name__)


class Verifier(Protocol):
    def verify(self, jpeg: bytes, target: str, frame_ts: float) -> VisionResult: ...
    def healthy(self) -> bool: ...


class VisionPipeline:
    def __init__(self, robot: RobotInterface, state: SharedState, verifier: Verifier) -> None:
        self.robot = robot
        self.state = state
        self.verifier = verifier
        self.tracker = TemplateTracker()
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_vlm = 0.0
        self._last_health = 0.0
        self._consecutive_no = 0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="vision", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.wait(config.VISION_TRACK_INTERVAL_S):
            try:
                self._step()
            except Exception:
                log.exception("vision step failed")

    def _step(self) -> None:
        now = time.time()
        if now - self._last_health > config.MODEL_HEALTH_INTERVAL_S:
            self._last_health = now
            self.state.set_health("vision_model", "ok" if self.verifier.healthy() else "down")

        target = self.state.target()
        if target is None or self.state.recall_active():
            if self.tracker.active:
                self.tracker.drop()
                self.state.set_tracker(False, None)
            return

        frame, frame_ts = self.robot.latest_frame()
        if frame is None:
            return

        # cheap tracker step
        if self.tracker.active:
            bbox = self.tracker.update(frame)
            self.state.set_tracker(self.tracker.active, bbox)
            if bbox is not None:
                prev = self.state.vision()
                self.state.set_vision(VisionResult(match=True, confidence=prev.confidence, bbox=bbox,
                                                   ts=now, frame_ts=frame_ts, ok=True,
                                                   bearing_deg=bearing_from_bbox(bbox), source="tracker"))

        # periodic identity verification
        if now - self._last_vlm >= config.VISION_VLM_INTERVAL_S:
            self._last_vlm = now
            jpeg, jts = self.robot.latest_jpeg(config.VISION_VLM_INPUT_WIDTH, config.VISION_VLM_JPEG_QUALITY)
            if jpeg is None:
                return
            res = self.verifier.verify(jpeg, target, jts)
            if res.match and res.bbox:
                self._consecutive_no = 0
                self.tracker.init(frame, res.bbox)
                self.state.set_tracker(True, res.bbox)
                self.state.set_vision(res)
            else:
                self._consecutive_no += 1
                if res.ok and self._consecutive_no >= 2 and self.tracker.active:
                    # two clean "no" verdicts beat a drifting tracker
                    self.tracker.drop()
                    self.state.set_tracker(False, None)
                    log_event(log, "vision.tracker_dropped", reason="vlm_denied_twice")
                if not self.tracker.active:
                    self.state.set_vision(res)
