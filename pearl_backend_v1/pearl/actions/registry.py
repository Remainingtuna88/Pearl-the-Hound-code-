"""
Goal registry — in-memory record of every goal ever submitted this run.

ROS2 does not keep goal history; the dashboard depends on this. Bounded by
config.GOAL_REGISTRY_MAX (>= 50). Thread-safe; returns dict snapshots.
"""
from __future__ import annotations

import threading
import time
from collections import OrderedDict
from typing import Any, Optional

from .. import config
from ..logging_setup import get_logger, log_event
from .goal import Goal, TERMINAL

log = get_logger(__name__)


class GoalRegistry:
    def __init__(self, max_size: int = config.GOAL_REGISTRY_MAX) -> None:
        self._lock = threading.RLock()
        self._goals: "OrderedDict[str, Goal]" = OrderedDict()
        self._max = max_size

    def add(self, goal: Goal) -> None:
        with self._lock:
            self._goals[goal.id] = goal
            # Evict oldest *terminal* goals first; never evict a live goal.
            while len(self._goals) > self._max:
                victim = next((gid for gid, g in self._goals.items() if g.state in TERMINAL), None)
                if victim is None:
                    break
                del self._goals[victim]
        log_event(log, "goal.submitted", goal_id=goal.id, action=goal.action, params=goal.params,
                  priority=goal.priority, source=goal.source, ttl=goal.ttl_seconds)

    def get(self, goal_id: str) -> Optional[Goal]:
        with self._lock:
            return self._goals.get(goal_id)

    def transition(self, goal: Goal, new_state: str, reason: Optional[str] = None,
                   detail: Optional[str] = None) -> None:
        """The ONLY place goal.state changes. Logs every transition."""
        with self._lock:
            old = goal.state
            goal.state = new_state
            now = time.time()
            if new_state == "EXECUTING" and goal.started_at is None:
                goal.started_at = now
            if new_state in TERMINAL:
                goal.ended_at = now
                goal.reason = reason
                goal.detail = detail
        log_event(log, "goal.transition", goal_id=goal.id, action=goal.action, frm=old, to=new_state,
                  reason=reason, detail=detail, priority=goal.priority)

    def live(self) -> list[Goal]:
        with self._lock:
            return [g for g in self._goals.values() if g.state not in TERMINAL]

    def executing(self) -> list[Goal]:
        with self._lock:
            return [g for g in self._goals.values() if g.state == "EXECUTING"]

    def pending(self) -> list[Goal]:
        with self._lock:
            return [g for g in self._goals.values() if g.state in ("PENDING", "ACCEPTED")]

    def history(self, n: int) -> list[dict[str, Any]]:
        with self._lock:
            term = [g for g in self._goals.values() if g.state in TERMINAL]
        term.sort(key=lambda g: g.ended_at or 0, reverse=True)
        return [g.to_dict() for g in term[:n]]

    def recent_outcomes(self, n: int) -> list[dict[str, Any]]:
        """Compact form for the planner context."""
        return [{"action": h["action"], "state": h["state"], "reason": h["reason"]}
                for h in self.history(n)]

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            active = [g.to_dict() for g in self._goals.values() if g.state == "EXECUTING"]
            pending = [g.to_dict() for g in self._goals.values() if g.state in ("PENDING", "ACCEPTED")]
        pending.sort(key=lambda d: (-d["priority"], d["submitted_at"]))
        return {
            "timestamp": time.time(),
            "active": active,
            "pending": pending,
            "history": self.history(config.GOAL_HISTORY_API_N),
            "registry_size": len(self._goals),
        }
