"""
ActionManager — Layer 3 core.

Responsibilities (none of these exist in ROS2; they are implemented here):

  1. Rejection at submission   validation.validate() runs before any motion.
                               Rejected goals still enter the registry with a
                               specific reason string.
  2. Resource mutexes          chassis and pan_tilt are independent locks. A
                               goal starts only when every resource it needs is
                               free or held by a LOWER-priority goal.
  3. Priority preemption       a higher-priority goal arriving for a held
                               resource preempts the holder synchronously,
                               inside submit(), before returning. No planner
                               round-trip involved.
  4. Queueing                  equal/lower priority conflicting goals wait as
                               PENDING (bounded; overflow -> queue_full).
                               Promotion happens on every tick, highest
                               priority first, FIFO within a priority.
  5. Goal TTL                  evidence_ts + ttl_seconds passes -> ABORTED
                               ttl_expired (executing or pending).
  6. Feedback                  executors write goal.feedback; visible to the
                               planner and the dashboard through the registry.
  7. Recall                    operator stop: cancel everything, block the
                               planner, issue a system-priority hold.

Threading: a single tick thread (ACTION_TICK_HZ) steps executors and does TTL
and promotion. submit()/recall() may be called from any thread; all state
mutation is under self._lock. Executor callbacks from the robot adapter only
write into plain attributes that the tick thread reads (single-writer).

Safety floor (Layer 4) is NOT here and cannot be reached from here; it only
notifies us after the fact via on_safety_stop() so goals get an honest reason.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Optional

from .. import config
from ..logging_setup import get_logger, log_event
from ..robot.interface import RobotInterface
from ..state import SharedState
from . import goal as G
from .executors import EXECUTORS, ExecContext, Executor
from .registry import GoalRegistry
from .validation import validate

log = get_logger(__name__)


class ActionManager:
    def __init__(self, robot: RobotInterface, state: SharedState, registry: Optional[GoalRegistry] = None) -> None:
        self.robot = robot
        self.state = state
        self.registry = registry or GoalRegistry()
        self._lock = threading.RLock()
        self._executors: dict[str, Executor] = {}          # goal_id -> executor (EXECUTING goals)
        self._holders: dict[str, str] = {}                 # resource -> goal_id
        self._pending: list[G.Goal] = []
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        self._thread = threading.Thread(target=self._loop, name="action-tick", daemon=True)
        self._thread.start()

    def shutdown(self) -> None:
        self._stop.set()
        with self._lock:
            for g in list(self.registry.executing()):
                self._finish(g, G.CANCELED, G.R_SHUTDOWN, None)
            for g in list(self._pending):
                self._finish(g, G.CANCELED, G.R_SHUTDOWN, None)

    def _loop(self) -> None:
        period = 1.0 / config.ACTION_TICK_HZ
        while not self._stop.wait(period):        # Event.wait => no busy loop
            try:
                self.tick()
            except Exception:                     # never let the tick thread die
                log.exception("action tick failed")

    # ------------------------------------------------------------------ submission
    def submit(self, goal: G.Goal) -> G.Goal:
        spec = G.ACTIONS.get(goal.action)
        goal.resources = frozenset(spec["resources"]) if spec else frozenset()
        if goal.ttl_seconds is None and spec is not None:
            goal.ttl_seconds = goal.params.get("ttl_seconds") or config.GOAL_DEFAULT_TTL_S.get(goal.action)

        with self._lock:
            self.registry.add(goal)
            sighting = self.state.last_sighting()
            rej = validate(
                goal,
                battery_pct=self.state.battery_pct(),
                recall_active=self.state.recall_active(),
                target_set=self.state.target() is not None,
                has_recent_sighting=bool(sighting and time.time() - sighting["ts"] < config.VISION_SIGHTING_STALE_S),
                pose_known=self.robot.pose() is not None,
            )
            if rej is None and self._is_duplicate(goal):
                rej = (G.R_DUPLICATE, "identical goal already live")
            if rej is not None:
                self.registry.transition(goal, G.REJECTED, rej[0], rej[1])
                log_event(log, "goal.rejected", goal_id=goal.id, action=goal.action, reason=rej[0], detail=rej[1])
                return goal

            # already expired on arrival? (evidence older than TTL)
            if self._ttl_expired(goal):
                self.registry.transition(goal, G.ABORTED, G.R_TTL_EXPIRED, "expired before start")
                return goal

            decision = self._arbitrate(goal)
            if decision == "start":
                self._start(goal)
            elif decision == "queue":
                if self._pending_count_for(goal.resources) >= config.GOAL_QUEUE_MAX:
                    self.registry.transition(goal, G.REJECTED, G.R_QUEUE_FULL,
                                             f"> {config.GOAL_QUEUE_MAX} goals waiting for {sorted(goal.resources)}")
                else:
                    self._pending.append(goal)
                    log_event(log, "goal.queued", goal_id=goal.id, action=goal.action, priority=goal.priority,
                              blocked_by=[self._holders[r] for r in goal.resources if r in self._holders])
            return goal

    def _is_duplicate(self, goal: G.Goal) -> bool:
        for g in self.registry.live():
            if g is not goal and g.action == goal.action and g.params == goal.params:
                return True
        return False

    def _pending_count_for(self, resources: frozenset[str]) -> int:
        return sum(1 for g in self._pending if g.resources & resources)

    def _arbitrate(self, goal: G.Goal) -> str:
        """'start' if every needed resource is free or held by strictly lower priority, else 'queue'."""
        for r in goal.resources:
            holder_id = self._holders.get(r)
            if holder_id is None:
                continue
            holder = self.registry.get(holder_id)
            if holder is None or holder.is_terminal():
                continue
            if goal.priority <= holder.priority:
                return "queue"
        return "start"

    def _start(self, goal: G.Goal) -> None:
        # Preempt whoever holds our resources (they are all lower priority here).
        for r in goal.resources:
            holder_id = self._holders.get(r)
            if holder_id and holder_id != goal.id:
                holder = self.registry.get(holder_id)
                if holder and not holder.is_terminal():
                    log_event(log, "goal.preempted", victim=holder.id, victim_priority=holder.priority,
                              by=goal.id, by_priority=goal.priority, resource=r)
                    self._finish(holder, G.ABORTED, G.R_PREEMPTED, f"by {goal.action}#{goal.id} p{goal.priority}")
        # Any chassis goal other than a concealment move breaks cover.
        if config.RESOURCE_CHASSIS in goal.resources and goal.action != "reposition_for_concealment":
            if self.state.concealment()["active"]:
                self.state.set_concealment(False)
        for r in goal.resources:
            self._holders[r] = goal.id
        self.registry.transition(goal, G.ACCEPTED)
        self.registry.transition(goal, G.EXECUTING)
        ex = EXECUTORS[goal.action]()
        self._executors[goal.id] = ex
        try:
            ex.start(ExecContext(self.robot, self.state, goal))
        except Exception as e:
            log.exception("executor start failed")
            self._finish(goal, G.ABORTED, G.R_EXECUTOR_ERROR, repr(e))

    def _finish(self, goal: G.Goal, state: str, reason: str, detail: Optional[str]) -> None:
        ex = self._executors.pop(goal.id, None)
        if ex is not None:
            try:
                ex.cancel(ExecContext(self.robot, self.state, goal))
            except Exception:
                log.exception("executor cancel failed")
        for r, gid in list(self._holders.items()):
            if gid == goal.id:
                del self._holders[r]
        if goal in self._pending:
            self._pending.remove(goal)
        if not goal.is_terminal():
            self.registry.transition(goal, state, reason, detail)
        # Reaching a concealment point puts the robot into cover.
        if goal.action == "reposition_for_concealment" and goal.state == G.SUCCEEDED:
            pt = goal.feedback.get("concealment_point") or {}
            spot = f"({pt.get('x')}, {pt.get('y')})" if pt else "in cover"
            self.state.set_concealment(True, spot=spot)

    # ------------------------------------------------------------------ tick
    def _ttl_expired(self, goal: G.Goal) -> bool:
        dl = goal.ttl_deadline()
        return dl is not None and time.time() > dl

    def tick(self) -> None:
        with self._lock:
            now = time.time()
            for goal in list(self.registry.executing()):
                ex = self._executors.get(goal.id)
                if ex is None:
                    self._finish(goal, G.ABORTED, G.R_EXECUTOR_ERROR, "executor missing")
                    continue
                if self._ttl_expired(goal):
                    self._finish(goal, G.ABORTED, G.R_TTL_EXPIRED,
                                 f"evidence {now - goal.evidence_ts:.0f}s old > ttl {goal.ttl_seconds}s")
                    continue
                goal.feedback["elapsed_s"] = round(now - (goal.started_at or now), 1)
                try:
                    term = ex.tick(ExecContext(self.robot, self.state, goal))
                except Exception as e:
                    log.exception("executor tick failed")
                    term = (G.ABORTED, G.R_EXECUTOR_ERROR, repr(e))
                if term:
                    self._finish(goal, *term)

            for goal in list(self._pending):
                if self._ttl_expired(goal):
                    self._finish(goal, G.ABORTED, G.R_TTL_EXPIRED, "expired while queued")

            # promote: highest priority first, FIFO within priority
            for goal in sorted(self._pending, key=lambda g: (-g.priority, g.submitted_at)):
                if self._arbitrate(goal) == "start" and all(r not in self._holders for r in goal.resources):
                    self._pending.remove(goal)
                    self._start(goal)

    # ------------------------------------------------------------------ operator / system entry points
    def recall(self) -> G.Goal:
        """Operator recall/stop: cancel everything, block the planner, hold."""
        with self._lock:
            self.state.set_recall(True)
            self.state.reset_behavior()
            self.state.set_concealment(False)
            for g in list(self.registry.executing()):
                self._finish(g, G.CANCELED, G.R_RECALLED, None)
            for g in list(self._pending):
                self._finish(g, G.CANCELED, G.R_RECALLED, None)
            self.robot.publish_cmd_vel(0.0, 0.0, 0.0)
            # TODO: if config.HOME_POSE is set, drive home instead of holding.
            hold = G.Goal(action="hold_position", params={"seconds": 120.0}, priority=config.PRIORITY_RECALL,
                          source=G.SOURCE_SYSTEM, ttl_seconds=None)
            return self.submit(hold)

    def cancel_all(self, reason: str = G.R_CANCELED_BY_OPERATOR) -> None:
        with self._lock:
            for g in list(self.registry.executing()) + list(self._pending):
                self._finish(g, G.CANCELED, reason, None)

    def on_safety_stop(self, active: bool, reason: Optional[str]) -> None:
        """Called by the safety floor AFTER it has already stopped the chassis.
        We abort chassis goals so the planner/dashboard see an honest reason."""
        if not active:
            return
        with self._lock:
            for g in list(self.registry.executing()):
                if config.RESOURCE_CHASSIS in g.resources:
                    self._finish(g, G.ABORTED, G.R_SAFETY_STOP, reason)

    # ------------------------------------------------------------------ views
    def active_goal(self) -> Optional[G.Goal]:
        ex = [g for g in self.registry.executing() if config.RESOURCE_CHASSIS in g.resources]
        return ex[0] if ex else None

    def snapshot(self) -> dict[str, Any]:
        return self.registry.snapshot()

    def planner_view(self) -> dict[str, Any]:
        """Compact goal info for the planner prompt."""
        act = self.registry.executing()
        return {
            "active": [{"action": g.action, "params": g.params, "priority": g.priority,
                        "elapsed_s": g.feedback.get("elapsed_s"),
                        "pct": g.feedback.get("percent_complete"),
                        "dist_m": g.feedback.get("distance_remaining_m")} for g in act],
            "pending": len(self._pending),
            "recent": self.registry.recent_outcomes(config.PLANNER_RECENT_OUTCOMES_N),
        }
