"""
PlannerClient — llama.cpp serving LFM2.5-230M.

`decide(context) -> PlannerDecision` with a list of validated ToolCall objects.
Never raises. On timeout / dead server / unparseable output it returns
ok=False and an empty call list; the loop (planner/loop.py) then applies the
configured safe fallback.

Output parsing order:
  1. native tool_calls in the response (mode "native")
  2. message.content as JSON {"tool_calls":[...]} (modes json_schema / gbnf / none)
  3. lenient: strip fences, find outer braces
Then every call is checked against actions/goal.py ACTIONS schema — the
grammar is a first line of defence, the validator is the second.
"""
from __future__ import annotations

import json
import re
import time
from dataclasses import dataclass, field
from typing import Any, Optional

import requests

from .. import config
from ..actions.goal import ACTIONS
from ..actions.validation import _check_schema
from ..logging_setup import get_logger, log_event
from . import prompts, tools

log = get_logger(__name__)

_FENCE = re.compile(r"```(?:json)?\s*(.*?)\s*```", re.S)
_BRACES = re.compile(r"\{.*\}", re.S)


@dataclass
class ToolCall:
    name: str
    arguments: dict[str, Any]
    priority: int


@dataclass
class PlannerDecision:
    ok: bool
    calls: list[ToolCall] = field(default_factory=list)
    error: Optional[str] = None
    raw: Optional[str] = None
    latency_ms: int = 0
    prompt_version: str = config.PLANNER_PROMPT_VERSION
    dropped: list[str] = field(default_factory=list)   # calls that failed validation, with why


def parse_planner_output(text: Optional[str], native_calls: Optional[list[dict[str, Any]]] = None
                         ) -> tuple[list[ToolCall], list[str], Optional[str]]:
    """Returns (valid_calls, dropped_reasons, fatal_error)."""
    raw_calls: list[dict[str, Any]] = []
    if native_calls:
        for c in native_calls:
            fn = c.get("function", {})
            args = fn.get("arguments", {})
            if isinstance(args, str):
                try:
                    args = json.loads(args)
                except json.JSONDecodeError:
                    args = {"__unparseable__": args}
            prio = args.pop("priority", None) if isinstance(args, dict) else None
            raw_calls.append({"name": fn.get("name"), "arguments": args, "priority": prio})
    elif text is not None:
        cand = text.strip()
        m = _FENCE.search(cand)
        if m:
            cand = m.group(1)
        try:
            obj = json.loads(cand)
        except json.JSONDecodeError:
            m2 = _BRACES.search(cand)
            if not m2:
                return [], [], f"no JSON in planner output: {text[:80]!r}"
            try:
                obj = json.loads(m2.group(0))
            except json.JSONDecodeError as e:
                return [], [], f"invalid JSON: {e}"
        if isinstance(obj, dict) and "tool_calls" in obj:
            raw_calls = obj["tool_calls"] if isinstance(obj["tool_calls"], list) else []
        elif isinstance(obj, dict) and "name" in obj:      # model emitted a single call
            raw_calls = [obj]
        elif isinstance(obj, list):
            raw_calls = obj
        else:
            return [], [], "JSON has no tool_calls"
    else:
        return [], [], "empty planner response"

    valid: list[ToolCall] = []
    dropped: list[str] = []
    for c in raw_calls[: config.PLANNER_MAX_CALLS_PER_CYCLE]:
        if not isinstance(c, dict):
            dropped.append("call is not an object")
            continue
        name = c.get("name")
        args = c.get("arguments", c.get("params", {})) or {}
        prio = c.get("priority", 20)
        if name not in ACTIONS:
            dropped.append(f"unknown tool {name!r}")
            continue
        if not isinstance(args, dict):
            dropped.append(f"{name}: arguments not an object")
            continue
        err = _check_schema(args, ACTIONS[name]["params"])
        if err:
            dropped.append(f"{name}: {err}")
            continue
        try:
            prio = int(prio)
        except (TypeError, ValueError):
            dropped.append(f"{name}: priority not an int")
            continue
        prio = max(config.PRIORITY_PLANNER_MIN, min(config.PRIORITY_PLANNER_MAX, prio))
        valid.append(ToolCall(name, args, prio))
    return valid, dropped, None


class PlannerClient:
    def __init__(self, url: str = config.PLANNER_URL, timeout: float = config.PLANNER_TIMEOUT_S) -> None:
        self.url = url
        self.timeout = timeout
        self._session = requests.Session()
        self._grammar: Optional[str] = None
        if config.PLANNER_CONSTRAIN_MODE == "gbnf":
            with open(config.PLANNER_GBNF_PATH, encoding="utf-8") as f:
                self._grammar = f.read()

    def _body(self, context: dict[str, Any]) -> dict[str, Any]:
        version = config.PLANNER_PROMPT_VERSION
        body: dict[str, Any] = {
            "model": config.PLANNER_MODEL_NAME,
            "temperature": config.PLANNER_TEMPERATURE,
            "max_tokens": config.PLANNER_MAX_TOKENS,
            "messages": [
                {"role": "system", "content": prompts.system_prompt(version)},
                {"role": "user", "content": json.dumps(context, separators=(",", ":"))},
            ],
        }
        mode = config.PLANNER_CONSTRAIN_MODE
        if mode == "json_schema":
            body["response_format"] = {"type": "json_schema",
                                       "json_schema": {"name": "plan", "schema": tools.RESPONSE_SCHEMA}}
        elif mode == "gbnf":
            body["grammar"] = self._grammar          # llama.cpp-specific extension
        elif mode == "native":
            body["tools"] = tools.openai_tools()
            body["tool_choice"] = "auto"
        return body

    def decide(self, context: dict[str, Any]) -> PlannerDecision:
        t0 = time.time()
        version = config.PLANNER_PROMPT_VERSION
        try:
            r = self._session.post(self.url, json=self._body(context), timeout=self.timeout)
            if r.status_code != 200:
                return self._fail(f"http {r.status_code}: {r.text[:120]}", t0)
            msg = r.json()["choices"][0]["message"]
        except requests.Timeout:
            return self._fail("timeout", t0)
        except requests.ConnectionError:
            return self._fail("connection_error (planner server down?)", t0)
        except (KeyError, IndexError, TypeError, ValueError) as e:
            return self._fail(f"unexpected response shape: {e}", t0)
        except Exception as e:
            return self._fail(f"unexpected: {e!r}", t0)

        text = msg.get("content")
        calls, dropped, fatal = parse_planner_output(text, msg.get("tool_calls"))
        dec = PlannerDecision(ok=fatal is None, calls=calls, error=fatal, raw=text,
                              latency_ms=int((time.time() - t0) * 1000), prompt_version=version, dropped=dropped)
        log_event(log, "planner.decision", ok=dec.ok, prompt_version=version, latency_ms=dec.latency_ms,
                  calls=[{"name": c.name, "args": c.arguments, "priority": c.priority} for c in calls],
                  dropped=dropped, error=fatal, context=context, raw=(text or "")[:300])
        return dec

    def _fail(self, error: str, t0: float) -> PlannerDecision:
        log_event(log, "planner.error", error=error, prompt_version=config.PLANNER_PROMPT_VERSION)
        return PlannerDecision(ok=False, error=error, latency_ms=int((time.time() - t0) * 1000))

    def healthy(self) -> bool:
        base = self.url.split("/v1/")[0]
        try:
            return self._session.get(base + config.MODEL_HEALTH_PATH, timeout=1.5).status_code == 200
        except requests.RequestException:
            return False
