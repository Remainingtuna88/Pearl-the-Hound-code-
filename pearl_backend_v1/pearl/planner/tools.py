"""
Planner tool vocabulary — derived from actions/goal.py ACTIONS so the planner
can never see an action the action layer does not implement, and the safety
floor (which is not in ACTIONS) is structurally invisible to it.

Three representations of the same thing:
  * openai_tools()      — OpenAI `tools` array (PLANNER_CONSTRAIN_MODE="native")
  * RESPONSE_SCHEMA     — JSON schema of {"tool_calls":[{name, arguments, priority}]}
                          for llama.cpp `response_format` (mode "json_schema")
  * grammar.gbnf        — hand-written GBNF equivalent (mode "gbnf")
"""
from __future__ import annotations

from typing import Any

from .. import config
from ..actions.goal import ACTIONS


def openai_tools() -> list[dict[str, Any]]:
    return [
        {"type": "function",
         "function": {"name": name, "description": spec["description"], "parameters": spec["params"]}}
        for name, spec in ACTIONS.items()
    ]


def _call_schema(name: str, spec: dict[str, Any]) -> dict[str, Any]:
    return {
        "type": "object",
        "properties": {
            "name": {"const": name},
            "arguments": spec["params"],
            "priority": {"type": "integer", "minimum": config.PRIORITY_PLANNER_MIN, "maximum": config.PRIORITY_PLANNER_MAX},
        },
        "required": ["name", "arguments", "priority"],
    }


RESPONSE_SCHEMA: dict[str, Any] = {
    "type": "object",
    "properties": {
        "tool_calls": {
            "type": "array",
            "minItems": 0,
            "maxItems": config.PLANNER_MAX_CALLS_PER_CYCLE,
            "items": {"anyOf": [_call_schema(n, s) for n, s in ACTIONS.items()]},
        }
    },
    "required": ["tool_calls"],
}


def vocabulary_text() -> str:
    """Compact human-readable list for the system prompt."""
    lines = []
    for name, spec in ACTIONS.items():
        props = spec["params"].get("properties", {})
        args = ", ".join(
            f"{k}:{v.get('type')}" + (f"∈{v['enum']}" if "enum" in v else "")
            for k, v in props.items()
        )
        lines.append(f"- {name}({args}) — {spec['description']}")
    return "\n".join(lines)
