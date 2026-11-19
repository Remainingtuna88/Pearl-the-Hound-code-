"""
Verify the bridge emits exactly what pearl_ui.html expects.

These replicate the dashboard's own validators (vState / vGoals / vVision) and
the specific nested field accesses its render functions make, so a contract
drift fails here instead of as a blank panel on the laptop.
"""
import json
import time

import pytest

from pearl import config
from pearl.actions import goal as G
from pearl.actions.goal import Goal
from pearl.bridge import dashboard_adapter as dash
from pearl.state import Pose2D, VisionResult


def is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


# ---- the dashboard's own validators, transcribed -----------------------------
def vState(o):
    assert isinstance(o, dict) and is_num(o["timestamp"])
    assert isinstance(o["mission"], dict)
    assert is_num(o["battery"]["percent"])
    assert isinstance(o["components"], dict)


def vGoals(o):
    assert isinstance(o, dict) and is_num(o["timestamp"])
    assert "active" in o
    assert isinstance(o["queue"], list) and isinstance(o["history"], list)


def vVision(o):
    assert isinstance(o, dict) and is_num(o["timestamp"])
    assert isinstance(o["match"], bool) and is_num(o["confidence"])


def _ms_or_none(v):
    assert v is None or (isinstance(v, int) and v > 1_000_000_000_000)  # ms epoch, not seconds


# ---- /api/state --------------------------------------------------------------
def test_state_payload_shape(world):
    state, robot, manager = world
    state.set_pose(Pose2D(1.0, 2.0, 0.0))
    state.set_battery(80.0, 12.0)
    p = dash.state_payload(state, manager)
    vState(p)
    m = p["mission"]
    # every field the dashboard's renderMission touches
    assert "target" in m
    _ms_or_none(m["target_set_at"])
    assert isinstance(m["zones"], list)
    for z in m["zones"]:
        assert set(z) >= {"id", "status"} and z["status"] in ("pending", "searching", "cleared")
    assert m["behavior_mode"] in ("follow", "conceal")
    assert set(m["concealment"]) >= {"active", "since", "spot", "cover_quality"}
    # the five fixed component keys with the right sub-shape
    assert set(p["components"]) == {"ros2_core", "planner_llm", "vision_llm", "nav2", "camera"}
    for c in p["components"].values():
        assert c["status"] in ("ok", "degraded", "down", "unknown")
        _ms_or_none(c["since"]); _ms_or_none(c["last_seen"])
    _ms_or_none(p["battery"]["timestamp"])
    json.dumps(p)


def test_state_zone_status_reflects_progress(world):
    state, robot, manager = world
    state.set_battery(80, 12)
    state.mark_zone_searched("zone_a")
    manager.submit(Goal("navigate_to_zone", {"zone_id": "zone_b"}, 20))
    p = dash.state_payload(state, manager)
    byid = {z["id"]: z["status"] for z in p["mission"]["zones"]}
    assert byid["zone_a"] == "cleared"
    assert byid["zone_b"] == "searching"
    assert byid["zone_c"] == "pending"


def test_last_sighting_has_map_coords(world):
    state, robot, manager = world
    state.set_pose(Pose2D(1.0, 1.0, 0.0))
    state.set_vision(VisionResult(match=True, confidence=0.9, bbox=[0.4, 0.4, 0.6, 0.6],
                                  ts=time.time(), ok=True, bearing_deg=0.0))
    p = dash.state_payload(state, manager)
    ls = p["mission"]["last_sighting"]
    assert ls is not None and is_num(ls["map_x"]) and is_num(ls["map_y"])
    assert ls["zone"] in config.ZONES and is_num(ls["confidence"])
    _ms_or_none(ls["timestamp"])


# ---- /api/goals --------------------------------------------------------------
def test_goals_active_is_single_object_or_null(world):
    state, robot, manager = world
    state.set_battery(80, 12); state.set_target("x")
    empty = dash.goals_payload(manager)
    vGoals(empty)
    assert empty["active"] is None            # the dashboard checks `g && g.active`

    manager.submit(Goal("navigate_to_zone", {"zone_id": "zone_a"}, 20))
    p = dash.goals_payload(manager)
    vGoals(p)
    a = p["active"]
    assert isinstance(a, dict)                 # NOT a list
    for k in ("id", "action", "params", "priority", "status", "submitted_at", "ttl_deadline"):
        assert k in a
    assert a["status"] == "EXECUTING"
    _ms_or_none(a["submitted_at"]); _ms_or_none(a["started_at"])


def test_goals_queue_key_and_preempted_status(world):
    state, robot, manager = world
    state.set_battery(80, 12); state.set_target("x")
    low = manager.submit(Goal("navigate_to_zone", {"zone_id": "zone_a"}, 20))
    manager.submit(Goal("navigate_to_zone", {"zone_id": "zone_b"}, 60))   # preempts low
    p = dash.goals_payload(manager)
    assert isinstance(p["queue"], list)
    statuses = {h["id"]: h["status"] for h in p["history"]}
    assert statuses[low.id] == "PREEMPTED"     # not ABORTED
    # history reason is present for non-succeeded terminals
    for h in p["history"]:
        if h["status"] in ("ABORTED", "REJECTED", "CANCELED", "PREEMPTED"):
            assert h["reason"]


def test_goals_history_uses_finished_at_ms(world):
    state, robot, manager = world
    state.set_battery(80, 12); state.set_target("x")
    r = manager.submit(Goal("fly", {}, 20))     # rejected -> terminal immediately
    p = dash.goals_payload(manager)
    h = [x for x in p["history"] if x["id"] == r.id][0]
    assert h["status"] == "REJECTED"
    _ms_or_none(h["finished_at"])


# ---- /api/vision -------------------------------------------------------------
def test_vision_bbox_is_object(world):
    state, robot, manager = world
    state.set_vision(VisionResult(match=True, confidence=0.8, bbox=[0.1, 0.2, 0.5, 0.8],
                                  ts=time.time(), ok=True))
    p = dash.vision_payload(state)
    vVision(p)
    assert set(p["bbox"]) == {"x_min", "y_min", "x_max", "y_max"}
    assert p["bbox"]["x_min"] == 0.1 and p["bbox"]["x_max"] == 0.5
    _ms_or_none(p["timestamp"])


def test_vision_no_match_is_valid(world):
    state, robot, manager = world
    p = dash.vision_payload(state)
    vVision(p)
    assert p["match"] is False and p["bbox"] is None


# ---- behavior ---------------------------------------------------------------
def test_behavior_roundtrips_into_state(world):
    state, robot, manager = world
    assert state.set_behavior("conceal") is True
    assert dash.state_payload(state, manager)["mission"]["behavior_mode"] == "conceal"
    assert state.set_behavior("lurk") is False   # unknown mode rejected


def test_concealment_set_on_reposition_success(world):
    state, robot, manager = world
    state.set_battery(80, 12); state.set_target("x")
    state.set_pose(Pose2D(0, 0, 0))
    g = manager.submit(Goal("reposition_for_concealment", {}, 40))
    assert g.state == G.EXECUTING
    robot.complete_nav(-1)
    manager.tick()
    assert g.state == G.SUCCEEDED
    conceal = dash.state_payload(state, manager)["mission"]["concealment"]
    assert conceal["active"] is True
