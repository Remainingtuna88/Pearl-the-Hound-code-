"""
Rejection at submission — every check runs BEFORE any motion.

`validate(goal, ctx)` returns None (accept) or a (reason, detail) tuple.
Reasons are specific strings from actions/goal.py, never booleans.
"""
from __future__ import annotations

from typing import Any, Optional

from .. import config
from ..geometry import point_in_polygon
from . import goal as G


def _check_schema(params: dict[str, Any], schema: dict[str, Any]) -> Optional[str]:
    """Tiny JSON-schema subset: object / required / type / enum / min / max."""
    if not isinstance(params, dict):
        return "params must be an object"
    props = schema.get("properties", {})
    for req in schema.get("required", []):
        if req not in params:
            return f"missing required param '{req}'"
    for key, val in params.items():
        if key not in props:
            return f"unexpected param '{key}'"
        spec = props[key]
        t = spec.get("type")
        if t == "number" and not (isinstance(val, (int, float)) and not isinstance(val, bool)):
            return f"'{key}' must be a number"
        if t == "string" and not isinstance(val, str):
            return f"'{key}' must be a string"
        if "enum" in spec and val not in spec["enum"]:
            return f"'{key}'={val!r} not in {spec['enum']}"
        if t == "number":
            if "minimum" in spec and val < spec["minimum"]:
                return f"'{key}'={val} below minimum {spec['minimum']}"
            if "maximum" in spec and val > spec["maximum"]:
                return f"'{key}'={val} above maximum {spec['maximum']}"
    return None


def validate(goal: G.Goal, *, battery_pct: Optional[float], recall_active: bool,
             target_set: bool, has_recent_sighting: bool, pose_known: bool) -> Optional[tuple[str, str]]:
    spec = G.ACTIONS.get(goal.action)
    if spec is None:
        return G.R_UNKNOWN_ACTION, f"no action named '{goal.action}'"

    err = _check_schema(goal.params, spec["params"])
    if err:
        return G.R_INVALID_PARAMS, err

    if goal.source == G.SOURCE_PLANNER and not (config.PRIORITY_PLANNER_MIN <= goal.priority <= config.PRIORITY_PLANNER_MAX):
        return G.R_PRIORITY_OUT_OF_RANGE, f"planner priority must be {config.PRIORITY_PLANNER_MIN}..{config.PRIORITY_PLANNER_MAX}"

    if recall_active and goal.source != G.SOURCE_SYSTEM:
        return G.R_RECALL_ACTIVE, "operator recall is active; only system goals accepted"

    if spec["motion"]:
        if battery_pct is not None and battery_pct < config.BATTERY_MIN_PERCENT_FOR_MOTION:
            return G.R_INSUFFICIENT_BATTERY, f"battery {battery_pct:.0f}% < {config.BATTERY_MIN_PERCENT_FOR_MOTION:.0f}%"
        if goal.source == G.SOURCE_PLANNER and not target_set:
            return G.R_NO_TARGET, "no target set — planner may not move the robot"

    if goal.action == "navigate_to_zone":
        zone = goal.params["zone_id"]
        if zone not in config.ZONES:
            return G.R_UNKNOWN_ZONE, f"zone '{zone}' not in config.ZONES"
        x, y, _ = config.ZONES[zone]
        if not point_in_polygon(x, y, config.GEOFENCE_POLYGON):
            return G.R_OUTSIDE_GEOFENCE, f"zone '{zone}' centre ({x}, {y}) is outside the geofence"

    if goal.action == "approach_target":
        if not has_recent_sighting:
            return G.R_NO_SIGHTING, f"no sighting within {config.VISION_SIGHTING_STALE_S:.0f}s"
        if not pose_known:
            return G.R_NO_POSE, "robot pose unknown; cannot project approach point"

    if goal.action == "reposition_for_concealment":
        if not config.CONCEALMENT_POINTS:
            return G.R_INVALID_PARAMS, "no concealment points configured"
        if not pose_known:
            return G.R_NO_POSE, "robot pose unknown; cannot pick nearest concealment point"
        for (x, y, _) in config.CONCEALMENT_POINTS:
            if not point_in_polygon(x, y, config.GEOFENCE_POLYGON):
                return G.R_OUTSIDE_GEOFENCE, f"concealment point ({x}, {y}) is outside the geofence (fix config)"

    if goal.action == "explore_until_found" and goal.source == G.SOURCE_PLANNER and not target_set:
        return G.R_NO_TARGET, "no target set"

    return None
