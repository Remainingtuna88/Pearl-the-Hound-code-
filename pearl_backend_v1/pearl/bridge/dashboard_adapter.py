"""
Dashboard adapter — translates the backend's internal snapshots into the exact
JSON contract the Pearl UI dashboard expects.

The dashboard's contract differs from the backend's internal representation in
several deliberate ways; this module is the single place those differences are
reconciled, so nothing internal has to bend to the UI:

  * timestamps: internal = float seconds; dashboard = integer MILLISECONDS.
  * /api/state.components: fixed keys ros2_core / planner_llm / vision_llm /
    nav2 / camera, each {status, since, last_seen}. Internal health uses names
    ros / planner_model / vision_model / nav2 / camera as flat strings.
  * /api/goals.active: a SINGLE goal object or null (internal keeps a list of
    executing goals — one per resource — so we pick the operative one).
  * /api/goals.queue: internal calls it "pending".
  * goal.status: internal uses `state`; an ABORTED goal whose reason is
    preempted_by_higher_priority is surfaced as the distinct PREEMPTED status
    the dashboard colours differently. ended_at -> finished_at.
  * /api/vision.bbox: object {x_min,y_min,x_max,y_max}; internal is a 4-array.

See pearl_ui.html's header comment for the authoritative contract.
"""
from __future__ import annotations

import time
from typing import Any, Optional

from .. import config


def ms(t: Optional[float]) -> Optional[int]:
    """Seconds -> integer milliseconds; None/0 -> None."""
    return int(t * 1000) if t else None


def now_ms() -> int:
    return int(time.time() * 1000)


# component key: dashboard name -> internal health key
_COMPONENT_MAP = [
    ("ros2_core", "ros"),
    ("planner_llm", "planner_model"),
    ("vision_llm", "vision_model"),
    ("nav2", "nav2"),
    ("camera", "camera"),
]


def state_payload(state, manager) -> dict[str, Any]:
    snap = state.snapshot_state()
    mission = snap["mission"]
    comps_raw = state.components_snapshot()

    components: dict[str, Any] = {}
    for dash_key, int_key in _COMPONENT_MAP:
        c = comps_raw.get(int_key, {"status": "unknown", "since": None, "last_seen": None})
        components[dash_key] = {"status": c["status"], "since": ms(c["since"]), "last_seen": ms(c["last_seen"])}

    # zone status: cleared (searched) / searching (active goal's current zone) / pending
    searched = set(snap["zones"]["searched"])
    searching = _searching_zone(manager)
    zones = [{"id": z, "status": "cleared" if z in searched else ("searching" if z == searching else "pending")}
             for z in config.ZONES]

    sight = snap["last_sighting"]
    last_sighting = None
    if sight:
        last_sighting = {
            "timestamp": ms(sight.get("ts")),
            "zone": sight.get("zone"),
            "map_x": sight.get("map_x"),
            "map_y": sight.get("map_y"),
            "confidence": sight.get("confidence"),
        }

    conceal = mission.get("concealment") or {"active": False, "since": None, "spot": None, "cover_quality": None}
    battery = snap["battery"]

    return {
        "timestamp": now_ms(),
        "mission": {
            "target": mission["target"],
            "target_set_at": ms(mission["target_set_at"]),
            "zones": zones,
            "last_sighting": last_sighting,
            "behavior_mode": mission.get("behavior_mode", "follow"),
            "behavior_set_at": ms(mission.get("behavior_set_at")),
            "concealment": {
                "active": conceal["active"],
                "since": ms(conceal["since"]),
                "spot": conceal["spot"],
                "cover_quality": conceal["cover_quality"],
            },
            # extra, ignored by the dashboard but handy for debugging:
            "state": mission["state"],
            "recall_active": mission["recall_active"],
        },
        "battery": {"percent": battery["percent"], "voltage": battery["voltage"], "timestamp": ms(battery["timestamp"])},
        "components": components,
        "stub_mode": snap["stub_mode"],
    }


def _searching_zone(manager) -> Optional[str]:
    for g in manager.registry.executing():
        if g.action in ("explore_until_found", "navigate_to_zone"):
            return g.feedback.get("current_zone") or g.feedback.get("zone_id")
    return None


def _goal_to_dash(d: dict[str, Any]) -> dict[str, Any]:
    """Transform a Goal.to_dict() into the dashboard goal shape."""
    status = d["state"]
    reason = d.get("reason")
    if status == "ABORTED" and reason == "preempted_by_higher_priority":
        status = "PREEMPTED"
    out = {
        "id": d["id"],
        "action": d["action"],
        "params": d["params"],
        "priority": d["priority"],
        "status": status,
        "submitted_at": ms(d.get("submitted_at")),
        "accepted_at": ms(d.get("started_at")),   # backend accepts+starts together
        "started_at": ms(d.get("started_at")),
        "finished_at": ms(d.get("ended_at")),
        "ttl_deadline": ms(d.get("ttl_deadline")),
        "reason": reason,
    }
    if d.get("feedback"):
        out["feedback"] = d["feedback"]
    return out


def _pick_active(actives: list[dict[str, Any]]) -> Optional[dict[str, Any]]:
    """The dashboard shows one active goal; prefer the chassis holder, else the
    highest priority."""
    if not actives:
        return None
    chassis = [a for a in actives if config.RESOURCE_CHASSIS in a.get("resources", [])]
    pool = chassis or actives
    return max(pool, key=lambda a: a["priority"])


def goals_payload(manager) -> dict[str, Any]:
    snap = manager.snapshot()
    active = _pick_active(snap["active"])
    return {
        "timestamp": now_ms(),
        "active": _goal_to_dash(active) if active else None,
        "queue": [_goal_to_dash(g) for g in snap["pending"]],
        "history": [_goal_to_dash(g) for g in snap["history"]],
    }


def vision_payload(state) -> dict[str, Any]:
    v = state.snapshot_vision()
    bbox = None
    if v["bbox"]:
        b = v["bbox"]
        bbox = {"x_min": b[0], "y_min": b[1], "x_max": b[2], "y_max": b[3]}
    # timestamp = when this inference finished (drives the UI's staleness clock)
    ts = v.get("vlm_timestamp") or v.get("frame_timestamp")
    return {
        "timestamp": ms(ts) or now_ms(),
        "match": bool(v["match"]),
        "confidence": float(v["confidence"]),
        "bbox": bbox,
        "model": config.VLM_MODEL_NAME,
        "source": v["source"],
        "ok": v["ok"],
        "error": v["error"],
    }
