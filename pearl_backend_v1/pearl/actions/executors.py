"""
Executors — one per action in the vocabulary.

An executor is a small non-blocking state machine. The ActionManager calls
`start()` once, then `tick()` at ACTION_TICK_HZ until it returns a terminal
tuple, or calls `cancel()`. No executor owns a thread; the Pi cannot afford
one thread per goal and the manager's single tick loop is easier to reason
about.

Navigation is delegated to Nav2's NavigateToPose through
RobotInterface.send_nav_goal (see robot/ros2_robot.py for the actual action
client; config.NAV2_ACTION_NAME / NAV2_ACTION_TYPE are where the real names go).
"""
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any, Optional

from .. import config
from ..geometry import dist, wrap_angle
from ..logging_setup import get_logger
from ..robot.interface import (RobotInterface, NavHandle, NavFeedback, NAV_SUCCEEDED, NAV_ABORTED,
                               NAV_CANCELED, NAV_REJECTED, NAV_SERVER_DOWN)
from ..state import SharedState
from . import goal as G

log = get_logger(__name__)

Terminal = tuple[str, str, Optional[str]]   # (state, reason, detail)


@dataclass
class ExecContext:
    robot: RobotInterface
    state: SharedState
    goal: G.Goal


class Executor:
    def start(self, ctx: ExecContext) -> None: ...
    def tick(self, ctx: ExecContext) -> Optional[Terminal]: return None
    def cancel(self, ctx: ExecContext) -> None: ...


# ---------------------------------------------------------------------------
# Nav2 helper shared by the driving executors
# ---------------------------------------------------------------------------
class _NavLeg:
    """One NavigateToPose goal with feedback plumbing into goal.feedback."""

    def __init__(self) -> None:
        self.handle: Optional[NavHandle] = None
        self.result: Optional[str] = None
        self.last_fb: Optional[NavFeedback] = None
        self.start_dist: Optional[float] = None
        self.sent_at: float = 0.0

    def send(self, ctx: ExecContext, x: float, y: float, yaw: float) -> None:
        self.result = None
        self.sent_at = time.time()
        pose = ctx.robot.pose()
        self.start_dist = dist(pose.x, pose.y, x, y) if pose else None

        def on_fb(fb: NavFeedback) -> None:
            self.last_fb = fb
            pct = None
            if fb.distance_remaining is not None and self.start_dist:
                pct = max(0.0, min(100.0, 100.0 * (1 - fb.distance_remaining / self.start_dist)))
            ctx.goal.feedback.update({
                "distance_remaining_m": None if fb.distance_remaining is None else round(fb.distance_remaining, 2),
                "percent_complete": None if pct is None else round(pct, 0),
                "elapsed_s": round(time.time() - (ctx.goal.started_at or time.time()), 1),
                "recoveries": fb.recoveries,
                "target": {"x": round(x, 2), "y": round(y, 2), "yaw": round(yaw, 2)},
            })

        def on_result(res: str) -> None:
            self.result = res

        self.handle = ctx.robot.send_nav_goal(x, y, yaw, on_fb, on_result)

    def cancel(self, ctx: ExecContext) -> None:
        if self.handle and not self.handle.done:
            ctx.robot.cancel_nav_goal(self.handle)

    def failure_terminal(self) -> Optional[Terminal]:
        """Map a non-success Nav2 result to a specific reason."""
        if self.result == NAV_ABORTED:
            # Nav2 does not tell us *why* in a machine-readable way; we make a
            # cheap guess from progress. TODO: read nav2 error codes (Humble
            # lacks them; Iron+ has result.error_code).
            fb = self.last_fb
            if fb and fb.distance_remaining is not None and self.start_dist and fb.distance_remaining > 0.7 * self.start_dist:
                return G.ABORTED, G.R_UNREACHABLE, "nav2 aborted with little progress"
            return G.ABORTED, G.R_PATH_BLOCKED, "nav2 aborted mid-route"
        if self.result == NAV_REJECTED:
            return G.ABORTED, G.R_NAV2_REJECTED, None
        if self.result == NAV_SERVER_DOWN:
            return G.ABORTED, G.R_NAV2_DOWN, "NavigateToPose action server unavailable"
        if self.result == NAV_CANCELED:
            return G.CANCELED, G.R_CANCELED_BY_OPERATOR, "nav2 goal canceled externally"
        return None


# ---------------------------------------------------------------------------
class NavigateToZone(Executor):
    def __init__(self) -> None:
        self.leg = _NavLeg()
        self.zone: str = ""

    def start(self, ctx: ExecContext) -> None:
        self.zone = ctx.goal.params["zone_id"]
        x, y, yaw = config.ZONES[self.zone]
        ctx.goal.feedback["zone_id"] = self.zone
        self.leg.send(ctx, x, y, yaw)

    def tick(self, ctx: ExecContext) -> Optional[Terminal]:
        if self.leg.result == NAV_SUCCEEDED:
            ctx.state.mark_zone_searched(self.zone)
            return G.SUCCEEDED, G.R_SUCCEEDED, f"reached {self.zone}"
        return self.leg.failure_terminal()

    def cancel(self, ctx: ExecContext) -> None:
        self.leg.cancel(ctx)


# ---------------------------------------------------------------------------
class ExploreUntilFound(Executor):
    """Visit unsearched zones nearest-first; stop as soon as vision matches."""

    def __init__(self) -> None:
        self.leg = _NavLeg()
        self.current: Optional[str] = None
        self.visited_this_goal: list[str] = []

    def _next_zone(self, ctx: ExecContext) -> Optional[str]:
        unsearched = ctx.state.zones_unsearched()
        if not unsearched:
            return None
        pose = ctx.robot.pose()
        if pose is None:
            return unsearched[0]
        return min(unsearched, key=lambda z: dist(pose.x, pose.y, config.ZONES[z][0], config.ZONES[z][1]))

    def _dispatch(self, ctx: ExecContext) -> Optional[Terminal]:
        z = self._next_zone(ctx)
        if z is None:
            return G.ABORTED, G.R_ZONES_EXHAUSTED, f"visited {self.visited_this_goal}"
        self.current = z
        x, y, yaw = config.ZONES[z]
        ctx.goal.feedback.update({"current_zone": z, "zones_left": ctx.state.zones_unsearched()})
        self.leg.send(ctx, x, y, yaw)
        return None

    def start(self, ctx: ExecContext) -> None:
        self._pending_terminal = self._dispatch(ctx)

    def tick(self, ctx: ExecContext) -> Optional[Terminal]:
        if getattr(self, "_pending_terminal", None):
            return self._pending_terminal
        v = ctx.state.vision()
        if v.match and v.confidence >= config.VISION_MATCH_CONFIDENCE_MIN:
            self.leg.cancel(ctx)
            return G.SUCCEEDED, G.R_TARGET_FOUND, f"match conf={v.confidence:.2f} near {self.current}"
        if self.leg.result == NAV_SUCCEEDED:
            ctx.state.mark_zone_searched(self.current)  # type: ignore[arg-type]
            self.visited_this_goal.append(self.current)  # type: ignore[arg-type]
            return self._dispatch(ctx)
        fail = self.leg.failure_terminal()
        if fail and fail[1] in (G.R_PATH_BLOCKED, G.R_UNREACHABLE):
            # Skip an unreachable zone rather than failing the whole exploration.
            log.warning("explore: zone %s unreachable (%s), skipping", self.current, fail[1])
            ctx.state.mark_zone_searched(self.current)  # type: ignore[arg-type]
            return self._dispatch(ctx)
        return fail

    def cancel(self, ctx: ExecContext) -> None:
        self.leg.cancel(ctx)


# ---------------------------------------------------------------------------
class ApproachTarget(Executor):
    """
    Project the last sighting into the map: bearing from bbox centre, range
    from the lidar bin at that bearing (fallback constant). Send a Nav2 goal
    `standoff` metres short of it; re-plan every APPROACH_REPLAN_INTERVAL_S
    while the target stays visible. Succeed when within standoff.

    TODO: calibrate CAMERA_HFOV_DEG and LIDAR_YAW_OFFSET_DEG; consider using
    the depth camera for range instead of lidar (better for people).
    """

    def __init__(self) -> None:
        self.leg = _NavLeg()
        self.last_plan = 0.0
        self.standoff = config.APPROACH_DEFAULT_STANDOFF_M

    def _plan(self, ctx: ExecContext) -> Optional[Terminal]:
        sighting = ctx.state.last_sighting()
        pose = ctx.robot.pose()
        if not sighting or pose is None:
            return G.ABORTED, G.R_TARGET_LOST, "no sighting/pose at plan time"
        bearing = math.radians(sighting.get("bearing_deg") or 0.0)
        rng = config.APPROACH_FALLBACK_RANGE_M
        lid = ctx.robot.lidar()
        if lid and lid.ranges_by_deg:
            key = int(round(math.degrees(bearing)))
            for k in (key, key - 1, key + 1, key - 2, key + 2):
                if k in lid.ranges_by_deg:
                    rng = lid.ranges_by_deg[k]
                    break
        # pose recorded with the sighting is where the robot was when it saw it
        rp = sighting.get("robot_pose") or {"x": pose.x, "y": pose.y, "yaw": pose.yaw}
        heading = rp["yaw"] + bearing
        tx = rp["x"] + rng * math.cos(heading)
        ty = rp["y"] + rng * math.sin(heading)
        travel = max(0.0, rng - self.standoff)
        gx = rp["x"] + travel * math.cos(heading)
        gy = rp["y"] + travel * math.sin(heading)
        ctx.goal.feedback.update({"target_est": {"x": round(tx, 2), "y": round(ty, 2)}, "range_est_m": round(rng, 2)})
        if dist(pose.x, pose.y, tx, ty) <= self.standoff + 0.15:
            return G.SUCCEEDED, G.R_SUCCEEDED, "within standoff"
        self.leg.send(ctx, gx, gy, wrap_angle(heading))
        self.last_plan = time.time()
        return None

    def start(self, ctx: ExecContext) -> None:
        self.standoff = float(ctx.goal.params.get("standoff_meters", config.APPROACH_DEFAULT_STANDOFF_M))
        self._pending_terminal = self._plan(ctx)

    def tick(self, ctx: ExecContext) -> Optional[Terminal]:
        if getattr(self, "_pending_terminal", None):
            return self._pending_terminal
        if self.leg.result == NAV_SUCCEEDED:
            return G.SUCCEEDED, G.R_SUCCEEDED, "reached standoff point"
        fail = self.leg.failure_terminal()
        if fail:
            return fail
        v = ctx.state.vision()
        if v.match and time.time() - self.last_plan > config.APPROACH_REPLAN_INTERVAL_S:
            self.leg.cancel(ctx)
            return self._plan(ctx)
        sighting = ctx.state.last_sighting()
        if sighting and time.time() - sighting["ts"] > config.VISION_SIGHTING_STALE_S:
            self.leg.cancel(ctx)
            return G.ABORTED, G.R_TARGET_LOST, "sighting went stale during approach"
        return None

    def cancel(self, ctx: ExecContext) -> None:
        self.leg.cancel(ctx)


# ---------------------------------------------------------------------------
class RepositionForConcealment(Executor):
    """v1: drive to the nearest configured concealment point. TODO: pick a point
    that actually breaks line-of-sight to the target using the map."""

    def __init__(self) -> None:
        self.leg = _NavLeg()

    def start(self, ctx: ExecContext) -> None:
        pose = ctx.robot.pose()
        pts = config.CONCEALMENT_POINTS
        x, y, yaw = min(pts, key=lambda p: dist(pose.x, pose.y, p[0], p[1])) if pose else pts[0]
        ctx.goal.feedback["concealment_point"] = {"x": x, "y": y}
        self.leg.send(ctx, x, y, yaw)

    def tick(self, ctx: ExecContext) -> Optional[Terminal]:
        if self.leg.result == NAV_SUCCEEDED:
            return G.SUCCEEDED, G.R_SUCCEEDED, "at concealment point"
        return self.leg.failure_terminal()

    def cancel(self, ctx: ExecContext) -> None:
        self.leg.cancel(ctx)


# ---------------------------------------------------------------------------
class HoldPosition(Executor):
    def __init__(self) -> None:
        self.until = 0.0

    def start(self, ctx: ExecContext) -> None:
        secs = float(ctx.goal.params.get("seconds", config.HOLD_DEFAULT_SECONDS))
        self.until = time.time() + secs
        ctx.robot.publish_cmd_vel(0.0, 0.0, 0.0)   # one explicit stop; Nav2 is idle here
        ctx.goal.feedback["hold_seconds"] = secs

    def tick(self, ctx: ExecContext) -> Optional[Terminal]:
        remaining = self.until - time.time()
        ctx.goal.feedback["remaining_s"] = round(max(0.0, remaining), 1)
        if remaining <= 0:
            return G.SUCCEEDED, G.R_SUCCEEDED, None
        return None


# ---------------------------------------------------------------------------
class ScanWithCamera(Executor):
    def __init__(self) -> None:
        self.until = 0.0

    def start(self, ctx: ExecContext) -> None:
        pan = float(ctx.goal.params["pan_deg"])
        tilt = float(ctx.goal.params.get("tilt_deg", 0.0))
        ctx.robot.set_pan_tilt(pan, tilt)
        ctx.goal.feedback.update({"pan_deg": pan, "tilt_deg": tilt})
        self.until = time.time() + config.PAN_TILT_SETTLE_S

    def tick(self, ctx: ExecContext) -> Optional[Terminal]:
        if time.time() >= self.until:
            return G.SUCCEEDED, G.R_SUCCEEDED, None
        return None


EXECUTORS: dict[str, type[Executor]] = {
    "navigate_to_zone": NavigateToZone,
    "explore_until_found": ExploreUntilFound,
    "approach_target": ApproachTarget,
    "reposition_for_concealment": RepositionForConcealment,
    "hold_position": HoldPosition,
    "scan_with_camera": ScanWithCamera,
}
