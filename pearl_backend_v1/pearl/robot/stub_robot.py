"""
Stub robot + stub model clients. No ROS2, no hardware, no model servers.

Everything here implements the same interfaces as the real components so the
action layer, safety floor, planner loop, vision pipeline and HTTP bridge run
unchanged. The synthetic world:

  * a 2-D robot that drives straight to nav goals at STUB_ROBOT_SPEED_MPS
  * a battery that drains slowly
  * a lidar that reads ~4 m ahead with random brief obstacles
  * a camera frame drawn with OpenCV: grid, pose text, and a red box for the
    target when it is within range and in front of the robot
  * StubVlm: says "match" when the red box is in frame (with a little noise
    and occasional simulated failures so the fail-closed paths get exercised)
  * StubPlanner: scripted policy mirroring planner/prompts.py v1
"""
from __future__ import annotations

import math
import random
import threading
import time
from typing import Any, Callable, Optional

import cv2
import numpy as np

from .. import config
from ..geometry import dist, wrap_angle
from ..logging_setup import get_logger
from ..planner.planner_client import PlannerDecision, ToolCall
from ..state import Pose2D, SharedState, VisionResult
from .interface import (RobotInterface, LidarSummary, NavFeedback, NavHandle,
                        NAV_SUCCEEDED, NAV_CANCELED)

log = get_logger(__name__)


class StubRobot(RobotInterface):
    def __init__(self, state: SharedState) -> None:
        self.state = state
        self._lock = threading.RLock()
        self._x, self._y, self._yaw = 0.0, 0.0, 0.0
        self._battery = 92.0
        self._gate_open = True
        self._lidar_cbs: list[Callable[[LidarSummary], None]] = []
        self._obstacle_ticks = 0
        self._nav: Optional[dict[str, Any]] = None
        self._frame_cache: tuple[Optional[np.ndarray], float] = (None, 0.0)
        self._pan, self._tilt = 0.0, 0.0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._counter = 0

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        for comp in ("ros", "camera", "lidar", "battery", "nav2"):
            self.state.set_health(comp, "ok", "stub")
        self._thread = threading.Thread(target=self._loop, name="stub-robot", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        dt = 1.0 / config.STUB_TICK_HZ
        while not self._stop.wait(dt):
            try:
                self._tick(dt)
            except Exception:
                log.exception("stub tick failed")

    def _tick(self, dt: float) -> None:
        with self._lock:
            self._battery = max(0.0, self._battery - config.STUB_BATTERY_DRAIN_PCT_PER_MIN * dt / 60.0)
            nav = self._nav
            if nav and self._gate_open:
                tx, ty, tyaw = nav["target"]
                d = dist(self._x, self._y, tx, ty)
                step = config.STUB_ROBOT_SPEED_MPS * dt
                if d <= max(step, 0.05):
                    self._x, self._y, self._yaw = tx, ty, tyaw
                    self._nav = None
                    nav["handle"].done = True
                    nav["handle"].result = NAV_SUCCEEDED
                    done_cb = nav["on_result"]
                else:
                    self._yaw = math.atan2(ty - self._y, tx - self._x)
                    self._x += step * math.cos(self._yaw)
                    self._y += step * math.sin(self._yaw)
                    done_cb = None
                fb = NavFeedback(distance_remaining=dist(self._x, self._y, tx, ty),
                                 navigation_time_s=time.time() - nav["sent"])
            else:
                fb, done_cb = None, None
            pose = Pose2D(self._x, self._y, self._yaw)

            # lidar: mostly clear, with random short obstacles
            if self._obstacle_ticks > 0:
                self._obstacle_ticks -= 1
                min_fwd = 0.2
            elif random.random() < config.STUB_OBSTACLE_PROBABILITY:
                self._obstacle_ticks = int(config.STUB_TICK_HZ * 1.5)
                min_fwd = 0.2
            else:
                min_fwd = 3.5 + random.random()
            visible, bbox, bearing, rng = self._target_view_locked()
            bins = {d: 4.0 for d in range(-30, 31, 5)}
            if visible and bearing is not None:
                bins[int(round(bearing))] = rng
            scan = LidarSummary(ts=time.time(), min_forward=min_fwd, ranges_by_deg=bins)

        # callbacks outside the lock (they take other locks)
        self.state.set_pose(pose)
        self._counter += 1
        if self._counter % int(config.STUB_TICK_HZ) == 0:
            self.state.set_battery(round(self._battery, 1), round(config.BATTERY_CELLS * 3.5 + self._battery / 100 * config.BATTERY_CELLS * 0.7, 2))
            # Heartbeat: refresh last_seen so the dashboard doesn't mark these
            # UNRESPONSIVE. set_health only logs on a real status change.
            for comp in ("ros", "camera", "lidar", "battery", "nav2"):
                self.state.set_health(comp, "ok", "stub")
        if nav:
            if fb:
                try:
                    nav["on_feedback"](fb)
                except Exception:
                    log.exception("nav feedback cb failed")
            if done_cb:
                try:
                    done_cb(NAV_SUCCEEDED)
                except Exception:
                    log.exception("nav result cb failed")
        for cb in self._lidar_cbs:
            try:
                cb(scan)
            except Exception:
                log.exception("lidar cb failed")

    # ------------------------------------------------------------------ synthetic target geometry
    def _target_view_locked(self) -> tuple[bool, Optional[list[float]], Optional[float], float]:
        tx, ty = config.STUB_TARGET_POSE
        rng = dist(self._x, self._y, tx, ty)
        bearing = math.degrees(wrap_angle(math.atan2(ty - self._y, tx - self._x) - self._yaw))
        half = config.CAMERA_HFOV_DEG / 2.0
        if rng > config.STUB_TARGET_VISIBLE_RANGE_M or abs(bearing) > half - 3:
            return False, None, None, rng
        cx = 0.5 - bearing / config.CAMERA_HFOV_DEG
        size = max(0.08, min(0.5, 0.35 / max(rng, 0.3)))
        bbox = [max(0.0, cx - size / 2), max(0.0, 0.5 - size), min(1.0, cx + size / 2), min(1.0, 0.5 + size)]
        return True, bbox, bearing, rng

    def target_view(self) -> tuple[bool, Optional[list[float]]]:
        with self._lock:
            vis, bbox, _, _ = self._target_view_locked()
            return vis, bbox

    # ------------------------------------------------------------------ sensing
    def latest_frame(self) -> tuple[Optional[np.ndarray], float]:
        frame, ts = self._frame_cache
        if frame is not None and time.time() - ts < 0.1:
            return frame, ts
        w, h = config.STUB_FRAME_SIZE
        img = np.full((h, w, 3), 70, dtype=np.uint8)
        for gx in range(0, w, 40):
            cv2.line(img, (gx, 0), (gx, h), (90, 90, 90), 1)
        for gy in range(0, h, 40):
            cv2.line(img, (0, gy), (w, gy), (90, 90, 90), 1)
        with self._lock:
            visible, bbox, _, rng = self._target_view_locked()
            txt = f"STUB x={self._x:.1f} y={self._y:.1f} yaw={math.degrees(self._yaw):.0f} bat={self._battery:.0f}%"
        if visible and bbox:
            cv2.rectangle(img, (int(bbox[0] * w), int(bbox[1] * h)), (int(bbox[2] * w), int(bbox[3] * h)), (40, 40, 220), -1)
            cv2.putText(img, f"target {rng:.1f}m", (int(bbox[0] * w), max(12, int(bbox[1] * h) - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv2.putText(img, txt, (6, h - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        cv2.putText(img, time.strftime("%H:%M:%S"), (w - 70, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1)
        self._frame_cache = (img, time.time())
        return self._frame_cache

    def latest_jpeg(self, max_width: int, quality: int) -> tuple[Optional[bytes], float]:
        frame, ts = self.latest_frame()
        if frame is None:
            return None, ts
        if frame.shape[1] > max_width:
            scale = max_width / frame.shape[1]
            frame = cv2.resize(frame, (max_width, int(frame.shape[0] * scale)), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return (buf.tobytes() if ok else None), ts

    def pose(self) -> Optional[Pose2D]:
        with self._lock:
            return Pose2D(self._x, self._y, self._yaw)

    def lidar(self) -> Optional[LidarSummary]:
        with self._lock:
            vis, _, bearing, rng = self._target_view_locked()
            bins = {d: 4.0 for d in range(-30, 31, 5)}
            if vis and bearing is not None:
                bins[int(round(bearing))] = rng
            return LidarSummary(ts=time.time(), min_forward=3.5, ranges_by_deg=bins)

    def register_lidar_callback(self, cb: Callable[[LidarSummary], None]) -> None:
        self._lidar_cbs.append(cb)

    # ------------------------------------------------------------------ motion
    def send_nav_goal(self, x, y, yaw, on_feedback, on_result) -> NavHandle:
        handle = NavHandle(f"stubnav-{int(time.time() * 1000) % 100000}")
        with self._lock:
            if self._nav:                       # Nav2 also preempts its own previous goal
                old = self._nav
                old["handle"].done, old["handle"].result = True, NAV_CANCELED
                self._nav = None
            else:
                old = None
            self._nav = {"target": (x, y, yaw), "handle": handle, "on_feedback": on_feedback,
                         "on_result": on_result, "sent": time.time()}
        if old:
            try:
                old["on_result"](NAV_CANCELED)
            except Exception:
                log.exception("old nav result cb failed")
        return handle

    def cancel_nav_goal(self, handle: NavHandle) -> None:
        with self._lock:
            nav = self._nav
            if nav and nav["handle"] is handle:
                self._nav = None
                handle.done, handle.result = True, NAV_CANCELED
            else:
                nav = None
        if nav:
            try:
                nav["on_result"](NAV_CANCELED)
            except Exception:
                log.exception("nav cancel cb failed")

    def publish_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        pass  # the stub only moves via nav goals

    def emergency_stop_publish(self) -> None:
        pass  # gate closed => _tick does not move

    def set_cmd_vel_gate(self, allow: bool) -> None:
        with self._lock:
            self._gate_open = allow

    def set_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None:
        with self._lock:
            self._pan, self._tilt = pan_deg, tilt_deg

    def nav2_available(self) -> bool:
        return True


# =============================================================================
class StubVlm:
    """Pretends to be the VLM. Uses the synthetic world's ground truth + noise."""

    def __init__(self, robot: StubRobot, failure_rate: float = 0.05) -> None:
        self.robot = robot
        self.failure_rate = failure_rate

    def verify(self, jpeg: bytes, target: str, frame_ts: float) -> VisionResult:
        time.sleep(config.STUB_VLM_LATENCY_S)
        if random.random() < self.failure_rate:
            return VisionResult(match=False, ok=False, error="stub: simulated model timeout",
                                ts=time.time(), frame_ts=frame_ts, source="stub")
        visible, bbox = self.robot.target_view()
        conf = random.uniform(0.7, 0.95) if visible else random.uniform(0.02, 0.3)
        match = visible and conf >= config.VISION_MATCH_CONFIDENCE_MIN
        bearing = None
        if match and bbox:
            bearing = round((0.5 - (bbox[0] + bbox[2]) / 2) * config.CAMERA_HFOV_DEG, 1)
        return VisionResult(match=match, confidence=round(conf, 2), bbox=bbox if match else None,
                            ts=time.time(), frame_ts=frame_ts, ok=True, bearing_deg=bearing, source="stub")

    def healthy(self) -> bool:
        return True


class StubPlanner:
    """Scripted policy standing in for LFM2.5-230M. Mirrors prompts.py v1."""

    def __init__(self, failure_rate: float = 0.02) -> None:
        self.failure_rate = failure_rate

    def decide(self, context: dict[str, Any]) -> PlannerDecision:
        time.sleep(config.STUB_PLANNER_LATENCY_S)
        if random.random() < self.failure_rate:
            return PlannerDecision(ok=False, error="stub: simulated planner timeout", prompt_version="stub")
        calls: list[ToolCall] = []
        goals = context.get("goals", {})
        active = goals.get("active", [])
        active_names = {a["action"] for a in active}
        vision = context.get("vision", {})
        bat = context.get("battery_pct")
        behavior = context.get("behavior", "follow")
        concealed = context.get("concealed", False)
        if bat is not None and bat < 20 and "reposition_for_concealment" not in active_names:
            calls.append(ToolCall("reposition_for_concealment", {}, 70))
        elif vision.get("match") and behavior == "conceal":
            if not concealed and "reposition_for_concealment" not in active_names:
                calls.append(ToolCall("reposition_for_concealment", {}, 60))
            elif concealed and not active:
                calls.append(ToolCall("hold_position", {"seconds": 5}, 30))       # watch from cover
        elif vision.get("match"):
            recent = goals.get("recent", [])
            just_arrived = recent and recent[0]["action"] == "approach_target" and recent[0]["state"] == "SUCCEEDED"
            if just_arrived and not active:
                calls.append(ToolCall("hold_position", {"seconds": 5}, 30))     # at standoff: watch, don't churn
            elif "approach_target" not in active_names and not just_arrived:
                calls.append(ToolCall("approach_target", {"standoff_meters": 1.5}, 60))
        elif context.get("last_sighting_age_s") is not None and context["last_sighting_age_s"] < 20:
            if "scan_with_camera" not in active_names:
                b = vision.get("bearing_deg") or 0.0
                calls.append(ToolCall("scan_with_camera", {"pan_deg": max(-90, min(90, b))}, 40))
        elif context.get("zones_unsearched"):
            if not active:
                calls.append(ToolCall("explore_until_found", {"ttl_seconds": 180}, 20))
        elif not active:
            calls.append(ToolCall("hold_position", {"seconds": 10}, 10))
        return PlannerDecision(ok=True, calls=calls, raw="stub", latency_ms=int(config.STUB_PLANNER_LATENCY_S * 1000),
                               prompt_version="stub")

    def healthy(self) -> bool:
        return True
