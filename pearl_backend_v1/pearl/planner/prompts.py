"""
Versioned planner system prompts.

The system prompt is a VERSIONED ARTIFACT: add a new key, never edit an old
one in place, and select with config.PLANNER_PROMPT_VERSION. Every
`planner.decision` log event records the version, so A/B runs can be
separated in the traces.

LFM2.5-230M is NOT a reasoning model. Keep prompts short, declarative,
structured; tell it the format, the vocabulary, and a handful of hard rules.
Avoid "think step by step" — it produces prose instead of tool calls.
"""
from __future__ import annotations

from .tools import vocabulary_text

_FORMAT = (
    'Output ONLY JSON: {"tool_calls":[{"name":..., "arguments":{...}, "priority":0-79}]}. '
    "0-2 calls. Empty list = keep doing the current goal."
)

PROMPT_VERSIONS: dict[str, str] = {
    # v1 — baseline
    "v1": (
        "You control a search robot. Choose the next action from the tools below using the state JSON.\n"
        "Tools:\n{tools}\n"
        "Rules:\n"
        "1. No target -> hold_position.\n"
        "2. Target set, not seen, zones_unsearched not empty -> explore_until_found (ttl 120-300).\n"
        "3. vision.match true AND behavior=follow -> approach_target standoff 1.5, priority 60.\n"
        "3b. vision.match true AND behavior=conceal AND not concealed -> reposition_for_concealment, priority 60.\n"
        "4. last_sighting_age_s < 20 but match false -> scan_with_camera toward bearing_deg.\n"
        "5. All zones searched and not found -> hold_position.\n"
        "6. battery_pct < 20 -> reposition_for_concealment then hold_position.\n"
        "7. If active goal is progressing (pct rising) and nothing changed -> empty list.\n"
        "Priority: routine 10-30, sighting-driven 50-70.\n"
        + _FORMAT
    ),
    # v1b — terser, for A/B against v1 on the small model
    "v1b": (
        "Search robot planner. Pick tools from:\n{tools}\n"
        "Policy: no target=hold. unseen+zones left=explore(ttl 180,p20). seen=approach(1.5,p60). "
        "recent sighting lost=scan toward bearing(p40). zones exhausted=hold. battery<20=conceal(p70). "
        "active goal progressing=[].\n" + _FORMAT
    ),
}


def system_prompt(version: str) -> str:
    if version not in PROMPT_VERSIONS:
        raise KeyError(f"unknown planner prompt version {version!r}; known: {list(PROMPT_VERSIONS)}")
    # str.replace, not str.format: the prompt text contains literal JSON braces.
    return PROMPT_VERSIONS[version].replace("{tools}", vocabulary_text())
