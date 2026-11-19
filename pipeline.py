"""
SafetyFloor — Layer 4. Lidar collision stop + geofence.

DESIGN INVARIANT (enforced by construction, not by convention):

    The planner has wide discretion over strategy and parameters, and ZERO
    discretion over the safety layer.

  * Not a tool. planner/tools.py derives its vocabulary from actions/goal.py
    ACTIONS; nothing in this module is registered there.
  * Not a goal. The floor does not go through ActionManager, has no priority,
    no TTL, cannot be preempted or cancelled. It has no public "disable".
  * Halts the chassis DIRECTLY via RobotInterface.emergency_stop_publish() and
    (preferred) by closing the cmd_vel gate — Nav2's velocity simply stops
    reaching the motors. Goal cancellation happens afterwards, for bookkeeping.
  * Fails closed: no lidar data for LIDAR_TIMEOUT_S => stop.
  * The stock Hiwonder obstacle avoidance is left running as an independent
    override; this floor is in addition to it, not a replacement.

Wiring on the real robot (TODO(hw), see config.SAFETY_GATE_CMD_VEL):
    Nav2 controller_server  --publishes-->  TOPICS.cmd_vel_nav
    Ros2Robot relay (gate)  --publishes-->  TOPICS.cmd_vel_motor  (only when safe)
If you cannot remap Nav2's output, set SAFETY_GATE_CMD_VEL=False: the floor
then publishes zeros to cmd_vel_motor at SAFETY_STOP_PUBLISH_HZ and fights
Nav2. That works in practice but is a last resort.

Threading: `on_scan()` is invoked from the robot adapter's lidar callback
(ROS executor thread or stub tick thread). A small watchdog thread handles the
"no scans arriving" case and the zero-velocity republish.
"""
from __future__ import annotations

import threading
import time
from typing import Callable, Optional

from .. import config
from ..geometry import point_in_polygon, distance_to_polygon_edge
from ..logging_setup import get_logger, log_event
from ..robot.interface import RobotInterface, LidarSummary
from ..state import SharedState

log = get_logger(__name__)

REASON_LIDAR_OBSTACLE = "lidar_obstacle"
REASON_LIDAR_TIMEOUT = "lidar_timeout"
REASON_GEOFENCE = "geofence"


class SafetyFloor:
    def __init__(self, robot: RobotInterface, state: SharedState,
                 on_stop_change: Optional[Callable[[bool, Optional[str]], None]] = None) -> None:
        self.robot = robot
        self.state = state
        self._on_stop_change = on_stop_change     # ActionManager.on_safety_stop
        self._lock = threading.Lock()
        self._stop_active = False
        self._reason: Optional[str] = None
        self._last_scan_ts = 0.0
        self._min_forward: Optional[float] = None
        self._halt = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        self.robot.register_lidar_callback(self.on_scan)
        self._thread = threading.Thread(target=self._watchdog, name="safety-floor", daemon=True)
        self._thread.start()
        log_event(log, "safety.started", gate=config.SAFETY_GATE_CMD_VEL,
                  stop_m=config.LIDAR_STOP_DISTANCE_M, resume_m=config.LIDAR_RESUME_DISTANCE_M)

    def shutdown(self) -> None:
        self._halt.set()

    # ------------------------------------------------------------------ inputs
    def on_scan(self, scan: LidarSummary) -> None:
        """Runs on the lidar callback — the fastest possible reaction path."""
        self._last_scan_ts = scan.ts
        self._min_forward = scan.min_forward
        self._evaluate()

    # ------------------------------------------------------------------ core
    def _evaluate(self) -> None:
        now = time.time()
        reason: Optional[str] = None

        # 1. lidar timeout — fail closed
        if now - self._last_scan_ts > config.LIDAR_TIMEOUT_S:
            reason = REASON_LIDAR_TIMEOUT
        else:
            # 2. obstacle in the forward arc, with hysteresis
            mf = self._min_forward
            if mf is not None:
                if mf < config.LIDAR_STOP_DISTANCE_M:
                    reason = REASON_LIDAR_OBSTACLE
                elif self._stop_active and self._reason == REASON_LIDAR_OBSTACLE and mf < config.LIDAR_RESUME_DISTANCE_M:
                    reason = REASON_LIDAR_OBSTACLE       # stay stopped until clear

        # 3. geofence on the robot's own pose
        if reason is None:
            pose = self.robot.pose()
            if pose is not None:
                inside = point_in_polygon(pose.x, pose.y, config.GEOFENCE_POLYGON)
                edge = distance_to_polygon_edge(pose.x, pose.y, config.GEOFENCE_POLYGON)
                if not inside or edge < config.GEOFENCE_MARGIN_M:
                    reason = REASON_GEOFENCE

        self._set_stop(reason is not None, reason)

    def _set_stop(self, active: bool, reason: Optional[str]) -> None:
        with self._lock:
            changed = active != self._stop_active
            self._stop_active = active
            self._reason = reason if active else None
        if active:
            # Always act, even if unchanged: this is the redundancy.
            self.robot.set_cmd_vel_gate(False)
            self.robot.emergency_stop_publish()
        if changed:
            if not active:
                self.robot.set_cmd_vel_gate(True)
            self.state.set_safety(active, reason)
            log_event(log, "safety.stop", active=active, reason=reason, min_forward=self._min_forward)
            if self._on_stop_change:
                try:
                    self._on_stop_change(active, reason)
                except Exception:
                    log.exception("on_stop_change callback failed")

    def _watchdog(self) -> None:
        period = 1.0 / config.SAFETY_STOP_PUBLISH_HZ
        while not self._halt.wait(period):
            try:
                if self._stop_active or time.time() - self._last_scan_ts > config.LIDAR_TIMEOUT_S:
                    self._evaluate()          # republishes zeros while active; catches timeouts
            except Exception:
                log.exception("safety watchdog failed")

    # ------------------------------------------------------------------ read-only view
    def is_stopped(self) -> bool:
        return self._stop_active
