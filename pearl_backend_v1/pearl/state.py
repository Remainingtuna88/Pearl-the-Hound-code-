"""
Thread-safe shared state.

Written by: ROS/stub robot adapter (battery, pose, sensor health), vision
pipeline (vision result), planner (last decision), action manager (mission
state, zones searched), safety floor (stop flag).

Read by: the HTTP bridge (dashboard), the planner (context), executors.

Rules:
  * Every public method takes the lock. Never hold the lock while doing I/O.
  * `snapshot_*()` methods return plain dicts (deep-copied) that can be
    serialised straight to JSON — the bridge never touches internal objects.
  * Health values are strings: "ok" | "degraded" | "down" | "unknown".
    Changes are logged as `health.change` events.
"""
from __future__ import annotations

import copy
import math
import threading
import time
from dataclasses import dataclass, field, asdict
from typing import Any, Optional

from . import config
from .logging_setup import get_logger, log_event

log = get_logger(__name__)

MISSION_IDLE = "IDLE"            # no target set
MISSION_SEARCHING = "SEARCHING"  # target set, no fresh sighting
MISSION_TRACKING = "TRACKING"    # fresh sighting
MISSION_RECALLED = "RECALLED"    # operator recall — planner disabled


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    ts: float = field(default_factory=time.time)


@dataclass
class VisionResult:
    """Output of Layer 1. `match` is the fail-closed verdict."""
    match: bool = False
    confidence: float = 0.0
    bbox: Optional[list[float]] = None      # [x_min, y_min, x_max, y_max] normalized 0-1
    ts: float = 0.0                          # when the VLM answered
    frame_ts: float = 0.0                    # when the frame was captured
    ok: bool = False                         # False => model error/timeout (still no match)
    error: Optional[str] = None
    bearing_deg: Optional[float] = None      # +left / -right of camera axis, from bbox centre
    source: str = "vlm"                      # "vlm" | "tracker" | "stub"


class SharedState:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._mission = MISSION_IDLE
        self._target: Optional[str] = None
        self._target_set_at: Optional[float] = None
        self._recall_active = False
        self._recall_at: Optional[float] = None
        self._battery_pct: Optional[float] = None
        self._battery_v: Optional[float] = None
        self._battery_ts: Optional[float] = None
        self._pose: Optional[Pose2D] = None
        self._vision = VisionResult()
        self._tracker: dict[str, Any] = {"active": False, "bbox": None, "ts": 0.0}
        self._last_sighting: Optional[dict[str, Any]] = None
        self._zones_searched: dict[str, float] = {}   # zone_id -> ts
        self._planner_last: dict[str, Any] = {}
        self._health: dict[str, str] = {
            "ros": "unknown", "camera": "unknown", "lidar": "unknown", "battery": "unknown",
            "nav2": "unknown", "vision_model": "unknown", "planner_model": "unknown",
        }
        # Per-component timing for the dashboard: `since` = when it entered its
        # current status; `last_seen` = last time we heard from it at all.
        self._health_since: dict[str, Optional[float]] = {k: None for k in self._health}
        self._health_seen: dict[str, Optional[float]] = {k: None for k in self._health}
        self._safety: dict[str, Any] = {"stop_active": False, "reason": None, "since": None}
        # Operator's standing instruction on contact with the target. Not a
        # command — the planner still chooses goals and the safety floor still
        # overrides. "follow" | "conceal".
        self._behavior_mode: str = "follow"
        self._behavior_set_at: Optional[float] = None
        # Is the robot currently holding in cover? Set when a
        # reposition_for_concealment goal succeeds; cleared when any other
        # chassis goal starts.
        self._concealment: dict[str, Any] = {"active": False, "since": None, "spot": None, "cover_quality": None}

    # ------------------------------------------------------------------ mission
    def set_target(self, description: str) -> None:
        with self._lock:
            self._target = description.strip()
            self._target_set_at = time.time()
            self._recall_active = False
            self._recall_at = None
            self._zones_searched.clear()
            self._last_sighting = None
            self._mission = MISSION_SEARCHING
        log_event(log, "mission.target_set", target=description)

    def set_recall(self, active: bool) -> None:
        with self._lock:
            self._recall_active = active
            self._recall_at = time.time() if active else None
            if active:
                self._mission = MISSION_RECALLED
        log_event(log, "mission.recall", active=active)

    def target(self) -> Optional[str]:
        with self._lock:
            return self._target

    def recall_active(self) -> bool:
        with self._lock:
            return self._recall_active

    def mission(self) -> str:
        with self._lock:
            self._refresh_mission_locked()
            return self._mission

    def _refresh_mission_locked(self) -> None:
        if self._recall_active:
            self._mission = MISSION_RECALLED
        elif self._target is None:
            self._mission = MISSION_IDLE
        elif self._last_sighting and time.time() - self._last_sighting["ts"] < config.VISION_SIGHTING_STALE_S:
            self._mission = MISSION_TRACKING
        else:
            self._mission = MISSION_SEARCHING

    # ------------------------------------------------------------------ sensors
    def set_battery(self, pct: Optional[float], volts: Optional[float]) -> None:
        with self._lock:
            self._battery_pct, self._battery_v, self._battery_ts = pct, volts, time.time()

    def battery_pct(self) -> Optional[float]:
        with self._lock:
            return self._battery_pct

    def set_pose(self, pose: Pose2D) -> None:
        with self._lock:
            self._pose = pose

    def pose(self) -> Optional[Pose2D]:
        with self._lock:
            return self._pose

    # ------------------------------------------------------------------ vision
    def set_vision(self, res: VisionResult) -> None:
        with self._lock:
            self._vision = res
            if res.match:
                map_x, map_y, zone = self._sighting_location_locked(res)
                self._last_sighting = {
                    "ts": res.ts, "confidence": res.confidence, "bbox": res.bbox,
                    "bearing_deg": res.bearing_deg,
                    "robot_pose": asdict(self._pose) if self._pose else None,
                    "map_x": map_x, "map_y": map_y, "zone": zone,
                }

    def _sighting_location_locked(self, res: VisionResult) -> tuple[Optional[float], Optional[float], Optional[str]]:
        """Approximate map position of the sighting and the nearest zone.

        The precise range comes later (approach uses lidar); here we just need a
        point for the dashboard. Project the bearing a nominal distance ahead of
        the robot, then label it with the nearest configured zone.
        """
        if self._pose is None:
            return None, None, None
        rng = config.APPROACH_FALLBACK_RANGE_M
        heading = self._pose.yaw + math.radians(res.bearing_deg or 0.0)
        mx = round(self._pose.x + rng * math.cos(heading), 2)
        my = round(self._pose.y + rng * math.sin(heading), 2)
        zone = None
        if config.ZONES:
            zone = min(config.ZONES, key=lambda z: (config.ZONES[z][0] - mx) ** 2 + (config.ZONES[z][1] - my) ** 2)
        return mx, my, zone

    def vision(self) -> VisionResult:
        with self._lock:
            return copy.copy(self._vision)

    def set_tracker(self, active: bool, bbox: Optional[list[float]]) -> None:
        with self._lock:
            self._tracker = {"active": active, "bbox": bbox, "ts": time.time()}

    def last_sighting(self) -> Optional[dict[str, Any]]:
        with self._lock:
            return copy.deepcopy(self._last_sighting)

    # ------------------------------------------------------------------ zones
    def mark_zone_searched(self, zone_id: str) -> None:
        with self._lock:
            self._zones_searched[zone_id] = time.time()
        log_event(log, "mission.zone_searched", zone=zone_id)

    def zones_searched(self) -> list[str]:
        with self._lock:
            return list(self._zones_searched.keys())

    def zones_unsearched(self) -> list[str]:
        with self._lock:
            return [z for z in config.ZONES if z not in self._zones_searched]

    # ------------------------------------------------------------------ planner
    def set_planner_last(self, info: dict[str, Any]) -> None:
        with self._lock:
            self._planner_last = dict(info, ts=time.time())

    # ------------------------------------------------------------------ behavior / concealment
    def set_behavior(self, mode: str) -> bool:
        """Operator behaviour on contact. Returns False for an unknown mode."""
        if mode not in ("follow", "conceal"):
            return False
        with self._lock:
            changed = mode != self._behavior_mode
            self._behavior_mode = mode
            self._behavior_set_at = time.time()
            if mode == "follow":                     # leaving conceal breaks cover
                self._concealment = {"active": False, "since": None, "spot": None, "cover_quality": None}
        if changed:
            log_event(log, "mission.behavior", mode=mode)
        return True

    def reset_behavior(self) -> None:
        """Recall returns the standing instruction to its default."""
        with self._lock:
            self._behavior_mode = "follow"
            self._behavior_set_at = None

    def behavior(self) -> str:
        with self._lock:
            return self._behavior_mode

    def set_concealment(self, active: bool, spot: Optional[str] = None, cover_quality: Optional[str] = None) -> None:
        with self._lock:
            if active:
                if not self._concealment["active"]:
                    self._concealment = {"active": True, "since": time.time(), "spot": spot,
                                         "cover_quality": cover_quality or "good"}
                else:
                    self._concealment["spot"] = spot or self._concealment["spot"]
            else:
                self._concealment = {"active": False, "since": None, "spot": None, "cover_quality": None}
        log_event(log, "mission.concealment", active=active, spot=spot)

    def concealment(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._concealment)

    # ------------------------------------------------------------------ health / safety
    def set_health(self, component: str, status: str, detail: Optional[str] = None) -> None:
        with self._lock:
            old = self._health.get(component)
            now = time.time()
            self._health[component] = status
            self._health_seen[component] = now
            if old != status:
                self._health_since[component] = now
        if old != status:
            log_event(log, "health.change", component=component, frm=old, to=status, detail=detail)

    def health(self) -> dict[str, str]:
        with self._lock:
            return dict(self._health)

    def components_snapshot(self) -> dict[str, dict[str, Any]]:
        """Per-component {status, since, last_seen} for the dashboard."""
        with self._lock:
            return {k: {"status": self._health[k], "since": self._health_since[k],
                        "last_seen": self._health_seen[k]} for k in self._health}

    def set_safety(self, stop_active: bool, reason: Optional[str]) -> None:
        with self._lock:
            was = self._safety["stop_active"]
            if stop_active and not was:
                self._safety = {"stop_active": True, "reason": reason, "since": time.time()}
            elif not stop_active:
                self._safety = {"stop_active": False, "reason": None, "since": None}
            else:
                self._safety["reason"] = reason

    def safety(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._safety)

    # ------------------------------------------------------------------ snapshots for the bridge
    def snapshot_state(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_mission_locked()
            return {
                "timestamp": time.time(),
                "mission": {
                    "state": self._mission,
                    "target": self._target,
                    "target_set_at": self._target_set_at,
                    "recall_active": self._recall_active,
                    "recall_at": self._recall_at,
                    "behavior_mode": self._behavior_mode,
                    "behavior_set_at": self._behavior_set_at,
                    "concealment": dict(self._concealment),
                },
                "battery": {"percent": self._battery_pct, "voltage": self._battery_v, "timestamp": self._battery_ts},
                "pose": asdict(self._pose) if self._pose else None,
                "zones": {
                    "all": list(config.ZONES.keys()),
                    "searched": list(self._zones_searched.keys()),
                    "searched_at": dict(self._zones_searched),
                },
                "last_sighting": copy.deepcopy(self._last_sighting),
                "health": dict(self._health),
                "safety": dict(self._safety),
                "planner": copy.deepcopy(self._planner_last),
                "stub_mode": config.STUB_MODE,
                "chassis": config.CHASSIS_VARIANT,
            }

    def snapshot_vision(self) -> dict[str, Any]:
        with self._lock:
            v = self._vision
            return {
                "timestamp": time.time(),
                "match": v.match,
                "confidence": v.confidence,
                "bbox": v.bbox,
                "bearing_deg": v.bearing_deg,
                "vlm_timestamp": v.ts,
                "frame_timestamp": v.frame_ts,
                "ok": v.ok,
                "error": v.error,
                "source": v.source,
                "target": self._target,
                "tracker": dict(self._tracker),
            }

    # planner context (compact — the 230M model has a small useful context)
    def planner_context(self) -> dict[str, Any]:
        with self._lock:
            self._refresh_mission_locked()
            v = self._vision
            age = round(time.time() - self._last_sighting["ts"], 1) if self._last_sighting else None
            return {
                "target": self._target,
                "mission": self._mission,
                "vision": {"match": v.match, "conf": round(v.confidence, 2), "bearing_deg": v.bearing_deg},
                "last_sighting_age_s": age,
                "zones_searched": list(self._zones_searched.keys()),
                "zones_unsearched": [z for z in config.ZONES if z not in self._zones_searched],
                "battery_pct": round(self._battery_pct, 0) if self._battery_pct is not None else None,
                "recall": self._recall_active,
                "safety_stop": self._safety["stop_active"],
                "behavior": self._behavior_mode,
                "concealed": self._concealment["active"],
            }
