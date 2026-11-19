"""
Goal model and vocabulary.

Lifecycle (mirrors the ROS2 action lifecycle, plus our extensions):

    PENDING -> ACCEPTED -> EXECUTING -> SUCCEEDED | ABORTED | CANCELED
            -> REJECTED   (validation failed at submission; no motion happened)

Extensions that are NOT ROS2 primitives (designed here, implemented in manager.py):
  * priority           — integer; higher preempts lower on a shared resource
  * ttl_seconds        — auto-abort when `evidence_ts + ttl` passes (reason ttl_expired)
  * resources          — mutexes this goal needs (chassis / pan_tilt)
  * reason             — specific string explaining any terminal state
"""
from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field, asdict
from typing import Any, Optional

from .. import config

# ---- states -----------------------------------------------------------------
PENDING = "PENDING"
ACCEPTED = "ACCEPTED"
REJECTED = "REJECTED"
EXECUTING = "EXECUTING"
SUCCEEDED = "SUCCEEDED"
ABORTED = "ABORTED"
CANCELED = "CANCELED"
TERMINAL = frozenset({REJECTED, SUCCEEDED, ABORTED, CANCELED})

# ---- reasons (keep this list in sync with README) ---------------------------
# rejection at submission
R_UNKNOWN_ACTION = "unknown_action"
R_INVALID_PARAMS = "invalid_params"
R_UNKNOWN_ZONE = "unknown_zone"
R_OUTSIDE_GEOFENCE = "outside_geofence"
R_INSUFFICIENT_BATTERY = "insufficient_battery"
R_RECALL_ACTIVE = "recall_active"
R_PRIORITY_OUT_OF_RANGE = "priority_out_of_range"
R_QUEUE_FULL = "queue_full"
R_DUPLICATE = "duplicate_goal"
R_NO_TARGET = "no_target_set"
R_NO_SIGHTING = "no_recent_sighting"
R_NO_POSE = "robot_pose_unknown"
# terminal reasons after acceptance
R_SUCCEEDED = "succeeded"
R_TTL_EXPIRED = "ttl_expired"
R_PREEMPTED = "preempted_by_higher_priority"
R_RECALLED = "recalled"
R_CANCELED_BY_OPERATOR = "canceled_by_operator"
R_PATH_BLOCKED = "path_blocked"
R_UNREACHABLE = "unreachable"
R_NAV2_REJECTED = "nav2_rejected_goal"
R_NAV2_DOWN = "nav2_server_down"
R_SAFETY_STOP = "safety_stop"
R_ZONES_EXHAUSTED = "all_zones_searched"
R_TARGET_LOST = "target_lost"
R_TARGET_FOUND = "target_found"
R_EXECUTOR_ERROR = "executor_exception"
R_SHUTDOWN = "shutdown"

SOURCE_PLANNER = "planner"
SOURCE_OPERATOR = "operator"
SOURCE_SYSTEM = "system"


@dataclass
class Goal:
    action: str
    params: dict[str, Any]
    priority: int
    source: str = SOURCE_PLANNER
    ttl_seconds: Optional[float] = None
    #: timestamp of the evidence that motivated the goal (a sighting, a
    #: planner cycle). TTL counts from here, not from submission.
    evidence_ts: float = field(default_factory=time.time)
    id: str = field(default_factory=lambda: uuid.uuid4().hex[:8])
    state: str = PENDING
    reason: Optional[str] = None
    detail: Optional[str] = None
    submitted_at: float = field(default_factory=time.time)
    started_at: Optional[float] = None
    ended_at: Optional[float] = None
    resources: frozenset[str] = field(default_factory=frozenset)
    feedback: dict[str, Any] = field(default_factory=dict)

    def is_terminal(self) -> bool:
        return self.state in TERMINAL

    def ttl_deadline(self) -> Optional[float]:
        return None if self.ttl_seconds is None else self.evidence_ts + self.ttl_seconds

    def to_dict(self) -> dict[str, Any]:
        d = asdict(self)
        d["resources"] = sorted(self.resources)
        d["ttl_deadline"] = self.ttl_deadline()
        now = time.time()
        d["elapsed_s"] = round(now - self.started_at, 1) if self.started_at and not self.ended_at else (
            round(self.ended_at - self.started_at, 1) if self.started_at and self.ended_at else None)
        return d


# ---- vocabulary -------------------------------------------------------------
# Each action: parameter schema (subset of JSON schema, validated by
# actions/validation.py) and required resources. The planner sees exactly this
# list as its tool set (planner/tools.py derives from it).

ACTIONS: dict[str, dict[str, Any]] = {
    "navigate_to_zone": {
        "description": "Drive to a named zone and mark it searched.",
        "params": {
            "type": "object",
            "properties": {"zone_id": {"type": "string", "enum": list(config.ZONES.keys())}},
            "required": ["zone_id"],
        },
        "resources": {config.RESOURCE_CHASSIS},
        "motion": True,
    },
    "explore_until_found": {
        "description": "Visit unsearched zones nearest-first until the target is seen or the TTL runs out.",
        "params": {
            "type": "object",
            "properties": {"ttl_seconds": {"type": "number", "minimum": 10, "maximum": 900}},
            "required": ["ttl_seconds"],
        },
        "resources": {config.RESOURCE_CHASSIS},
        "motion": True,
    },
    "approach_target": {
        "description": "Move toward the last sighting and stop at standoff_meters. Requires a recent sighting.",
        "params": {
            "type": "object",
            "properties": {"standoff_meters": {"type": "number", "minimum": 0.5, "maximum": 5.0}},
            "required": ["standoff_meters"],
        },
        "resources": {config.RESOURCE_CHASSIS},
        "motion": True,
    },
    "reposition_for_concealment": {
        "description": "Move to the nearest configured concealment point.",
        "params": {"type": "object", "properties": {}, "required": []},
        "resources": {config.RESOURCE_CHASSIS},
        "motion": True,
    },
    "hold_position": {
        "description": "Stay still for `seconds` (default 5).",
        "params": {
            "type": "object",
            "properties": {"seconds": {"type": "number", "minimum": 1, "maximum": 120}},
            "required": [],
        },
        "resources": {config.RESOURCE_CHASSIS},
        "motion": False,
    },
    "scan_with_camera": {
        "description": "Pan/tilt the camera (degrees). Legal while driving.",
        "params": {
            "type": "object",
            "properties": {
                "pan_deg": {"type": "number", "minimum": config.PAN_LIMIT_DEG[0], "maximum": config.PAN_LIMIT_DEG[1]},
                "tilt_deg": {"type": "number", "minimum": config.TILT_LIMIT_DEG[0], "maximum": config.TILT_LIMIT_DEG[1]},
            },
            "required": ["pan_deg"],
        },
        "resources": {config.RESOURCE_PAN_TILT},
        "motion": False,
    },
}
