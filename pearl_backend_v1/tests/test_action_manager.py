"""Layer 3 behaviour: rejection reasons, mutexes, preemption, TTL, queueing, recall, registry."""
import time

from pearl import config
from pearl.actions import goal as G
from pearl.actions.goal import Goal
from pearl.actions.registry import GoalRegistry
from pearl.state import Pose2D, VisionResult


def g(action, params=None, priority=20, **kw):
    return Goal(action=action, params=params or {}, priority=priority, **kw)


# ---------------------------------------------------------------- rejection at submission
def test_unknown_action_rejected(world):
    _, _, m = world
    r = m.submit(g("fly_away"))
    assert r.state == G.REJECTED and r.reason == G.R_UNKNOWN_ACTION


def test_invalid_params_rejected(world):
    _, _, m = world
    r = m.submit(g("navigate_to_zone", {"zone_id": "nowhere"}))
    assert r.state == G.REJECTED and r.reason == G.R_INVALID_PARAMS   # enum check fires first
    r = m.submit(g("approach_target", {"standoff_meters": 99}))
    assert r.reason == G.R_INVALID_PARAMS and "maximum" in r.detail


def test_outside_geofence_rejected(world, monkeypatch):
    _, _, m = world
    monkeypatch.setitem(config.ZONES, "zone_far", (100.0, 100.0, 0.0))
    monkeypatch.setitem(G.ACTIONS["navigate_to_zone"]["params"]["properties"]["zone_id"], "enum", list(config.ZONES))
    r = m.submit(g("navigate_to_zone", {"zone_id": "zone_far"}))
    assert r.state == G.REJECTED and r.reason == G.R_OUTSIDE_GEOFENCE


def test_insufficient_battery_rejects_motion_but_not_hold(world):
    state, _, m = world
    state.set_battery(5.0, 10.0)
    assert m.submit(g("navigate_to_zone", {"zone_id": "zone_a"})).reason == G.R_INSUFFICIENT_BATTERY
    assert m.submit(g("hold_position")).state == G.EXECUTING


def test_planner_priority_out_of_range(world):
    _, _, m = world
    assert m.submit(g("hold_position", priority=90)).reason == G.R_PRIORITY_OUT_OF_RANGE


def test_approach_requires_recent_sighting(world):
    state, _, m = world
    assert m.submit(g("approach_target", {"standoff_meters": 1.0})).reason == G.R_NO_SIGHTING
    state.set_vision(VisionResult(match=True, confidence=0.9, bbox=[0.4, 0.4, 0.6, 0.6], ts=time.time(), ok=True, bearing_deg=0.0))
    assert m.submit(g("approach_target", {"standoff_meters": 1.0})).state == G.EXECUTING


def test_no_target_blocks_planner_motion(world):
    state, robot, m = world
    from pearl.state import SharedState
    from pearl.actions.manager import ActionManager
    s2 = SharedState(); s2.set_battery(80, 12)
    m2 = ActionManager(robot, s2)
    assert m2.submit(g("navigate_to_zone", {"zone_id": "zone_a"})).reason == G.R_NO_TARGET


# ---------------------------------------------------------------- mutexes
def test_chassis_and_pan_tilt_are_independent(world):
    _, robot, m = world
    nav = m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}, priority=20))
    scan = m.submit(g("scan_with_camera", {"pan_deg": 30}, priority=20))
    assert nav.state == G.EXECUTING and scan.state == G.EXECUTING
    assert robot.pan_tilt == (30.0, 0.0)


def test_equal_priority_conflict_queues(world):
    _, robot, m = world
    a = m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}, priority=20))
    b = m.submit(g("navigate_to_zone", {"zone_id": "zone_b"}, priority=20))
    assert a.state == G.EXECUTING and b.state == G.PENDING
    assert len(robot.nav_goals) == 1                    # b never touched the chassis
    robot.complete_nav(0)
    m.tick()
    assert a.state == G.SUCCEEDED and b.state == G.EXECUTING
    assert len(robot.nav_goals) == 2


def test_queue_promotion_is_highest_priority_first(world):
    _, robot, m = world
    m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}, priority=30))
    low = m.submit(g("navigate_to_zone", {"zone_id": "zone_b"}, priority=10))
    mid = m.submit(g("navigate_to_zone", {"zone_id": "zone_c"}, priority=20))
    robot.complete_nav(0)
    m.tick()
    assert mid.state == G.EXECUTING and low.state == G.PENDING


def test_queue_full_rejected(world, monkeypatch):
    _, _, m = world
    monkeypatch.setattr(config, "GOAL_QUEUE_MAX", 1)
    m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}))
    m.submit(g("navigate_to_zone", {"zone_id": "zone_b"}))
    r = m.submit(g("navigate_to_zone", {"zone_id": "zone_c"}))
    assert r.state == G.REJECTED and r.reason == G.R_QUEUE_FULL


def test_duplicate_live_goal_rejected(world):
    _, _, m = world
    m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}))
    assert m.submit(g("navigate_to_zone", {"zone_id": "zone_a"})).reason == G.R_DUPLICATE


# ---------------------------------------------------------------- preemption
def test_higher_priority_preempts_immediately(world):
    _, robot, m = world
    low = m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}, priority=20))
    high = m.submit(g("navigate_to_zone", {"zone_id": "zone_b"}, priority=60))
    # synchronous: by the time submit() returns, the victim is terminal and its nav goal cancelled
    assert low.state == G.ABORTED and low.reason == G.R_PREEMPTED
    assert robot.nav_goals[0]["handle"].done
    assert high.state == G.EXECUTING
    assert m.active_goal() is high


def test_preempted_lower_goal_does_not_come_back(world):
    _, robot, m = world
    low = m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}, priority=20))
    high = m.submit(g("navigate_to_zone", {"zone_id": "zone_b"}, priority=60))
    robot.complete_nav(-1)
    m.tick()
    assert high.state == G.SUCCEEDED and low.state == G.ABORTED
    assert m.active_goal() is None


# ---------------------------------------------------------------- TTL
def test_ttl_expires_executing_goal(world):
    _, _, m = world
    goal = m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}, ttl_seconds=0.05))
    assert goal.state == G.EXECUTING
    time.sleep(0.08)
    m.tick()
    assert goal.state == G.ABORTED and goal.reason == G.R_TTL_EXPIRED


def test_ttl_counts_from_evidence_not_submission(world):
    state, _, m = world
    state.set_vision(VisionResult(match=True, confidence=0.9, bbox=[0.4, 0.4, 0.6, 0.6], ts=time.time(), ok=True, bearing_deg=0.0))
    stale = m.submit(g("approach_target", {"standoff_meters": 1.0}, evidence_ts=time.time() - 100, ttl_seconds=45))
    assert stale.state == G.ABORTED and stale.reason == G.R_TTL_EXPIRED


def test_ttl_expires_queued_goal(world):
    _, _, m = world
    m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}))
    queued = m.submit(g("navigate_to_zone", {"zone_id": "zone_b"}, ttl_seconds=0.05))
    assert queued.state == G.PENDING
    time.sleep(0.08)
    m.tick()
    assert queued.state == G.ABORTED and queued.reason == G.R_TTL_EXPIRED


def test_explore_ttl_comes_from_params(world):
    _, _, m = world
    e = m.submit(g("explore_until_found", {"ttl_seconds": 120}))
    assert e.ttl_seconds == 120


# ---------------------------------------------------------------- feedback & executors
def test_nav_feedback_streams_into_goal(world):
    _, robot, m = world
    from pearl.robot.interface import NavFeedback
    goal = m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}))
    robot.nav_goals[0]["fb"](NavFeedback(distance_remaining=0.5))
    m.tick()
    fb = goal.feedback
    assert fb["distance_remaining_m"] == 0.5 and fb["percent_complete"] == 50.0 and "elapsed_s" in fb


def test_explore_visits_zones_and_marks_searched(world):
    state, robot, m = world
    e = m.submit(g("explore_until_found", {"ttl_seconds": 300}))
    assert e.state == G.EXECUTING
    for _ in range(len(config.ZONES)):
        robot.complete_nav(-1)
        m.tick()
    assert e.state == G.ABORTED and e.reason == G.R_ZONES_EXHAUSTED
    assert set(state.zones_searched()) == set(config.ZONES)


def test_explore_stops_on_sighting(world):
    state, robot, m = world
    e = m.submit(g("explore_until_found", {"ttl_seconds": 300}))
    state.set_vision(VisionResult(match=True, confidence=0.9, bbox=[0.4, 0.4, 0.6, 0.6], ts=time.time(), ok=True))
    m.tick()
    assert e.state == G.SUCCEEDED and e.reason == G.R_TARGET_FOUND
    assert robot.nav_goals[0]["handle"].done            # nav goal was cancelled


def test_hold_position_completes(world):
    _, robot, m = world
    h = m.submit(g("hold_position", {"seconds": 1}))
    assert robot.cmd_vel[-1] == (0.0, 0.0, 0.0)
    time.sleep(1.05)
    m.tick()
    assert h.state == G.SUCCEEDED


# ---------------------------------------------------------------- recall & safety
def test_recall_cancels_everything_and_blocks_planner(world):
    state, robot, m = world
    a = m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}))
    b = m.submit(g("navigate_to_zone", {"zone_id": "zone_b"}))
    hold = m.recall()
    assert a.state == G.CANCELED and a.reason == G.R_RECALLED
    assert b.state == G.CANCELED and b.reason == G.R_RECALLED
    assert hold.state == G.EXECUTING and hold.priority == config.PRIORITY_RECALL
    assert state.recall_active()
    assert m.submit(g("navigate_to_zone", {"zone_id": "zone_c"})).reason == G.R_RECALL_ACTIVE
    state.set_target("new target")            # operator sets a new target => recall lifts
    assert m.submit(g("navigate_to_zone", {"zone_id": "zone_c"}), ).state == G.PENDING  # hold still holds chassis at p95


def test_safety_stop_aborts_chassis_goal_with_reason(world):
    _, _, m = world
    nav = m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}))
    scan = m.submit(g("scan_with_camera", {"pan_deg": 10}))
    m.on_safety_stop(True, "lidar_obstacle")
    assert nav.state == G.ABORTED and nav.reason == G.R_SAFETY_STOP and nav.detail == "lidar_obstacle"
    assert scan.state == G.EXECUTING          # pan-tilt is not a chassis resource


# ---------------------------------------------------------------- registry
def test_registry_keeps_history_and_evicts_terminal_first():
    reg = GoalRegistry(max_size=5)
    live = Goal("hold_position", {}, 1)
    reg.add(live)
    for i in range(10):
        gg = Goal("hold_position", {"seconds": i}, 1)
        reg.add(gg)
        reg.transition(gg, G.SUCCEEDED, G.R_SUCCEEDED)
    assert reg.get(live.id) is live
    snap = reg.snapshot()
    assert snap["registry_size"] <= 5 and len(snap["history"]) >= 4
    assert all(h["state"] in G.TERMINAL for h in snap["history"])


def test_goals_snapshot_is_json_serialisable(world):
    import json
    _, _, m = world
    m.submit(g("navigate_to_zone", {"zone_id": "zone_a"}))
    m.submit(g("fly"))
    json.dumps(m.snapshot())
