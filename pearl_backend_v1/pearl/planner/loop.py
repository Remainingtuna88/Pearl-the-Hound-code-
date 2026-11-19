"""
PlannerLoop — one thread, every PLANNER_INTERVAL_S:

    build compact context  ->  PlannerClient.decide()  ->  ActionManager.submit()

  * Does nothing while no target is set or recall is active (the planner has
    no business moving the robot then; validation would reject it anyway —
    this just saves the CPU).
  * On planner failure applies the safe fallback ONLY when no chassis goal is
    executing; an executing goal simply continues.
  * Never blocks the ROS executor or the HTTP server: it runs in its own
    thread and the model call has a hard timeout.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Optional, Protocol

from .. import config
from ..actions.goal import Goal, SOURCE_PLANNER
from ..actions.manager import ActionManager
from ..logging_setup import get_logger, log_event
from ..state import SharedState
from .planner_client import PlannerDecision

log = get_logger(__name__)


class Decider(Protocol):
    def decide(self, context: dict[str, Any]) -> PlannerDecision: ...
    def healthy(self) -> bool: ...


class PlannerLoop:
    def __init__(self, state: SharedState, manager: ActionManager, decider: Decider) -> None:
        self.state = state
        self.manager = manager
        self.decider = decider
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._last_health = 0.0
        self._cycle = 0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="planner", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()

    def _loop(self) -> None:
        while not self._stop.wait(config.PLANNER_INTERVAL_S):
            try:
                self.cycle()
            except Exception:
                log.exception("planner cycle failed")

    def build_context(self) -> dict[str, Any]:
        ctx = self.state.planner_context()
        ctx["goals"] = self.manager.planner_view()
        return ctx

    def cycle(self) -> None:
        now = time.time()
        if now - self._last_health > config.MODEL_HEALTH_INTERVAL_S:
            self._last_health = now
            self.state.set_health("planner_model", "ok" if self.decider.healthy() else "down")

        if self.state.target() is None or self.state.recall_active():
            return

        self._cycle += 1
        ctx = self.build_context()
        vision_ts = self.state.vision().ts
        dec = self.decider.decide(ctx)

        submitted = []
        if not dec.ok:
            if self.manager.active_goal() is None:
                g = Goal(action=config.PLANNER_FALLBACK_ACTION, params=dict(config.PLANNER_FALLBACK_PARAMS),
                         priority=config.PLANNER_FALLBACK_PRIORITY, source=SOURCE_PLANNER)
                self.manager.submit(g)
                submitted.append(f"{g.action}#{g.id}:{g.state}")
                log_event(log, "planner.fallback", error=dec.error, action=g.action)
        else:
            for call in dec.calls:
                # evidence timestamp: a sighting-driven goal ages with the sighting
                evidence = vision_ts if call.name in ("approach_target", "scan_with_camera") and vision_ts else now
                g = Goal(action=call.name, params=call.arguments, priority=call.priority,
                         source=SOURCE_PLANNER, evidence_ts=evidence)
                self.manager.submit(g)
                submitted.append(f"{g.action}#{g.id}:{g.state}" + (f"({g.reason})" if g.reason else ""))

        self.state.set_planner_last({
            "cycle": self._cycle, "ok": dec.ok, "error": dec.error, "latency_ms": dec.latency_ms,
            "prompt_version": dec.prompt_version,
            "calls": [{"name": c.name, "arguments": c.arguments, "priority": c.priority} for c in dec.calls],
            "dropped": dec.dropped, "submitted": submitted,
        })
