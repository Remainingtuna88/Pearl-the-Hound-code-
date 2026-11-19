"""
RobotInterface — the ONLY surface through which the rest of the backend talks
to hardware. Everything above this line (actions, planner, vision, safety,
bridge) is identical in stub and real mode.

Two implementations:
  robot/ros2_robot.py  — rclpy, real robot
  robot/stub_robot.py  — synthetic data, no ROS2 import anywhere

Design notes
  * Frame access returns the latest frame by reference; callers must not
    mutate it. Nothing copies image data unless it is about to encode it.
  * Nav goals are asynchronous with callbacks — the action layer never blocks.
  * `emergency_stop_publish()` is what the safety floor calls. It must reach
    the motors with the least possible latency and no dependency on the
    action system.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Callable, Optional, Any

import numpy as np

from ..state import Pose2D


@dataclass
class LidarSummary:
    ts: float
    min_forward: Optional[float]           # closest valid return in the forward arc, metres
    ranges_by_deg: dict[int, float] = field(default_factory=dict)  # coarse 1° bins, base frame


@dataclass
class NavFeedback:
    distance_remaining: Optional[float] = None
    navigation_time_s: Optional[float] = None
    recoveries: int = 0


#: Terminal results of a Nav2 goal as seen by the wrapper.
NAV_SUCCEEDED = "succeeded"
NAV_ABORTED = "aborted"       # Nav2 gave up (path blocked / unreachable)
NAV_CANCELED = "canceled"
NAV_REJECTED = "rejected"     # Nav2 refused the goal
NAV_SERVER_DOWN = "server_down"


class NavHandle:
    """Opaque handle for an in-flight navigation goal."""
    def __init__(self, ident: str):
        self.id = ident
        self.done = False
        self.result: Optional[str] = None
        self.raw: Any = None


class RobotInterface(ABC):
    # lifecycle --------------------------------------------------------------
    @abstractmethod
    def start(self) -> None: ...

    @abstractmethod
    def shutdown(self) -> None: ...

    # sensing ----------------------------------------------------------------
    @abstractmethod
    def latest_frame(self) -> tuple[Optional[np.ndarray], float]:
        """(BGR frame or None, capture timestamp). Decodes lazily; no copy."""

    @abstractmethod
    def latest_jpeg(self, max_width: int, quality: int) -> tuple[Optional[bytes], float]:
        """Downscaled JPEG bytes for the VLM / MJPEG stream."""

    @abstractmethod
    def pose(self) -> Optional[Pose2D]: ...

    @abstractmethod
    def lidar(self) -> Optional[LidarSummary]: ...

    @abstractmethod
    def register_lidar_callback(self, cb: Callable[[LidarSummary], None]) -> None:
        """Safety floor hooks here so it reacts on the scan callback itself."""

    # motion -----------------------------------------------------------------
    @abstractmethod
    def send_nav_goal(self, x: float, y: float, yaw: float,
                      on_feedback: Callable[[NavFeedback], None],
                      on_result: Callable[[str], None]) -> NavHandle: ...

    @abstractmethod
    def cancel_nav_goal(self, handle: NavHandle) -> None: ...

    @abstractmethod
    def publish_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        """Direct velocity. Used sparingly (safety zeros, tiny adjustments)."""

    @abstractmethod
    def emergency_stop_publish(self) -> None:
        """Zero velocity straight to the motor topic. Called at 10 Hz while a safety stop is active."""

    @abstractmethod
    def set_cmd_vel_gate(self, allow: bool) -> None:
        """If the adapter relays Nav2's cmd_vel to the motors, open/close that relay."""

    @abstractmethod
    def set_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None: ...

    # health -----------------------------------------------------------------
    @abstractmethod
    def nav2_available(self) -> bool: ...
