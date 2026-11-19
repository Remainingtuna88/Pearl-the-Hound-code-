"""Fail-closed parsing for the VLM and planner outputs; geometry helpers."""
from pearl.vision.vlm_client import parse_vlm_output, bearing_from_bbox
from pearl.planner.planner_client import parse_planner_output
from pearl.planner import prompts, tools
from pearl.geometry import point_in_polygon, distance_to_polygon_edge


# ---------------------------------------------------------------- VLM
def test_vlm_clean_json():
    p, err = parse_vlm_output('{"match": true, "confidence": 0.9, "bbox": [0.1, 0.2, 0.5, 0.8]}')
    assert err is None and p["match"] and p["bbox"] == [0.1, 0.2, 0.5, 0.8]


def test_vlm_markdown_fenced():
    p, err = parse_vlm_output('Sure!\n```json\n{"match": false, "confidence": 0.1, "bbox": null}\n```')
    assert err is None and p["match"] is False and p["bbox"] is None


def test_vlm_prose_wrapped():
    p, err = parse_vlm_output('The answer is {"match": "true", "confidence": "0.7", "bbox": [0,0,1,1]} ok')
    assert err is None and p["match"] and p["confidence"] == 0.7


def test_vlm_garbage_fails_closed():
    for bad in ["", "no json here", "[1,2,3]", '{"match": "maybe"}', '{"confidence": 1}', None]:
        p, err = parse_vlm_output(bad)  # type: ignore[arg-type]
        assert p is None and err


def test_vlm_bad_bbox_dropped_not_fatal():
    p, err = parse_vlm_output('{"match": true, "confidence": 0.8, "bbox": [0.9, 0.9, 0.1, 0.1]}')
    assert err is None and p["bbox"] is None
    p, _ = parse_vlm_output('{"match": true, "confidence": 5, "bbox": [0.1, 0.1, 0.2]}')
    assert p["confidence"] == 1.0 and p["bbox"] is None


def test_bearing():
    assert bearing_from_bbox([0.4, 0.0, 0.6, 1.0]) == 0.0
    assert bearing_from_bbox([0.0, 0.0, 0.2, 1.0]) > 0      # left of centre => positive
    assert bearing_from_bbox(None) is None


# ---------------------------------------------------------------- planner
def test_planner_json_tool_calls():
    calls, dropped, fatal = parse_planner_output(
        '{"tool_calls":[{"name":"explore_until_found","arguments":{"ttl_seconds":120},"priority":20}]}')
    assert fatal is None and not dropped
    assert calls[0].name == "explore_until_found" and calls[0].priority == 20


def test_planner_native_tool_calls():
    native = [{"function": {"name": "approach_target", "arguments": '{"standoff_meters": 1.5, "priority": 60}'}}]
    calls, dropped, fatal = parse_planner_output(None, native)
    assert fatal is None and calls[0].arguments == {"standoff_meters": 1.5} and calls[0].priority == 60


def test_planner_drops_invalid_keeps_valid():
    calls, dropped, fatal = parse_planner_output(
        '{"tool_calls":[{"name":"disable_safety","arguments":{},"priority":99},'
        '{"name":"navigate_to_zone","arguments":{"zone_id":"mars"},"priority":20},'
        '{"name":"hold_position","arguments":{},"priority":500}]}')
    assert fatal is None
    assert [c.name for c in calls] == []            # PLANNER_MAX_CALLS_PER_CYCLE=2 => third never examined
    assert len(dropped) == 2 and "unknown tool" in dropped[0] and "mars" in dropped[1]


def test_planner_priority_clamped():
    calls, _, _ = parse_planner_output('{"tool_calls":[{"name":"hold_position","arguments":{},"priority":500}]}')
    assert calls[0].priority == 79


def test_planner_empty_list_is_valid_noop():
    calls, dropped, fatal = parse_planner_output('{"tool_calls":[]}')
    assert fatal is None and calls == [] and dropped == []


def test_planner_garbage_is_fatal():
    calls, _, fatal = parse_planner_output("I think we should explore.")
    assert calls == [] and fatal


def test_safety_is_not_a_tool():
    names = {t["function"]["name"] for t in tools.openai_tools()}
    assert not any("safety" in n or "geofence" in n or "stop" in n for n in names)
    assert "safety" not in prompts.system_prompt("v1").lower()


def test_prompt_versions_render():
    for v in prompts.PROMPT_VERSIONS:
        assert "navigate_to_zone" in prompts.system_prompt(v)


# ---------------------------------------------------------------- geometry
def test_polygon():
    sq = [(0, 0), (2, 0), (2, 2), (0, 2)]
    assert point_in_polygon(1, 1, sq) and not point_in_polygon(3, 1, sq)
    assert abs(distance_to_polygon_edge(1, 1, sq) - 1.0) < 1e-9
    assert abs(distance_to_polygon_edge(1, 0.2, sq) - 0.2) < 1e-9
