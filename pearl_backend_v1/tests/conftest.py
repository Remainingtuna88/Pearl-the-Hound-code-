import os
import sys
import time
from typing import Callable, Optional

import numpy as np
import pytest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
os.environ.setdefault("PEARL_STUB", "1")

from pearl import config  # noqa: E402

# keep test noise out of the real trace log
config.LOG_JSONL_FILE = os.path.join(config.LOG_DIR, "test_run.jsonl")
from pearl.robot.interface import RobotInterface, LidarSummary, NavHandle, NavFeedback, NAV_SUCCEEDED, NAV_CANCELED  # noqa: E402
from pearl.state import SharedState, Pose2D  # noqa: E402
from pearl.actions.manager import ActionManager  # noqa: E402


class FakeRobot(RobotInterface):
    """Deterministic robot for unit tests: nav goals complete only when the test says so."""

    def __init__(self):
        self._pose = Pose2D(0.0, 0.0, 0.0)
        self.nav_goals = []          # list of dicts
        self.cmd_vel = []
        self.gate = True
        self.pan_tilt = None
        self._lidar_cbs = []

    def start(self): ...
    def shutdown(self): ...
    def latest_frame(self): return np.zeros((240, 320, 3), np.uint8), time.time()
    def latest_jpeg(self, max_width, quality): return b"\xff\xd8fake", time.time()
    def pose(self): return self._pose
    def lidar(self): return LidarSummary(ts=time.time(), min_forward=3.0, ranges_by_deg={0: 2.5})
    def register_lidar_callback(self, cb): self._lidar_cbs.append(cb)

    def send_nav_goal(self, x, y, yaw, on_feedback, on_result):
        h = NavHandle(f"fake{len(self.nav_goals)}")
        self.nav_goals.append({"target": (x, y, yaw), "handle": h, "fb": on_feedback, "res": on_result})
        return h

    def cancel_nav_goal(self, handle):
        for g in self.nav_goals:
            if g["handle"] is handle and not handle.done:
                handle.done, handle.result = True, NAV_CANCELED
                g["res"](NAV_CANCELED)

    def complete_nav(self, idx=-1):
        g = self.nav_goals[idx]
        g["fb"](NavFeedback(distance_remaining=0.0))
        g["handle"].done, g["handle"].result = True, NAV_SUCCEEDED
        self._pose = Pose2D(*g["target"])
        g["res"](NAV_SUCCEEDED)

    def publish_cmd_vel(self, vx, vy, wz): self.cmd_vel.append((vx, vy, wz))
    def emergency_stop_publish(self): self.cmd_vel.append((0.0, 0.0, 0.0))
    def set_cmd_vel_gate(self, allow): self.gate = allow
    def set_pan_tilt(self, pan, tilt): self.pan_tilt = (pan, tilt)
    def nav2_available(self): return True


@pytest.fixture
def world():
    state = SharedState()
    state.set_battery(80.0, 12.0)
    state.set_target("test target")
    robot = FakeRobot()
    manager = ActionManager(robot, state)     # not started: tests call manager.tick() manually
    return state, robot, manager
