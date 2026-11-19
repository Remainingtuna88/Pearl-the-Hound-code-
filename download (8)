"""
Wiring. Builds the five layers on top of one RobotInterface and starts the
threads in dependency order:

    robot adapter (ROS executor thread | stub tick thread)
      └─ safety floor            (lidar callback + watchdog thread)   Layer 4
      └─ action manager          (tick thread)                        Layer 3
      └─ vision pipeline         (thread)                             Layer 1
      └─ planner loop            (thread)                             Layer 2
      └─ HTTP bridge             (werkzeug thread)                    Layer 5
    main thread: waits for SIGINT/SIGTERM, then shuts down in reverse.

Thread inventory (so nobody has to guess): ros-executor (x2 workers) or
stub-robot, safety-floor, action-tick, vision, planner, http (+1 per request).
"""
from __future__ import annotations

import signal
import threading

from . import config
from .actions.manager import ActionManager
from .bridge.http_server import create_app, HttpServer
from .logging_setup import get_logger, log_event, setup_logging
from .planner.loop import PlannerLoop
from .safety.safety_floor import SafetyFloor
from .state import SharedState
from .vision.pipeline import VisionPipeline

log = get_logger(__name__)


def build():
    state = SharedState()

    if config.STUB_MODE:
        from .robot.stub_robot import StubRobot, StubVlm, StubPlanner
        robot = StubRobot(state)
        if config.STUB_USE_REAL_MODELS:
            from .vision.vlm_client import VlmClient
            from .planner.planner_client import PlannerClient
            verifier, decider = VlmClient(), PlannerClient()
        else:
            verifier, decider = StubVlm(robot), StubPlanner()
    else:
        from .robot.ros2_robot import Ros2Robot
        from .vision.vlm_client import VlmClient
        from .planner.planner_client import PlannerClient
        robot = Ros2Robot(state)
        verifier, decider = VlmClient(), PlannerClient()

    manager = ActionManager(robot, state)
    safety = SafetyFloor(robot, state, on_stop_change=manager.on_safety_stop)
    vision = VisionPipeline(robot, state, verifier)
    planner = PlannerLoop(state, manager, decider)
    http = HttpServer(create_app(state, manager, robot))
    return state, robot, safety, manager, vision, planner, http


def main() -> None:
    setup_logging()
    log_event(log, "backend.starting", stub_mode=config.STUB_MODE, chassis=config.CHASSIS_VARIANT,
              http_port=config.HTTP_PORT, prompt_version=config.PLANNER_PROMPT_VERSION)
    state, robot, safety, manager, vision, planner, http = build()

    robot.start()
    safety.start()
    manager.start()
    vision.start()
    planner.start()
    http.start()
    log_event(log, "backend.ready", threads=[t.name for t in threading.enumerate()])

    stop = threading.Event()

    def _sig(*_):
        stop.set()

    signal.signal(signal.SIGINT, _sig)
    signal.signal(signal.SIGTERM, _sig)
    stop.wait()

    log_event(log, "backend.stopping")
    for comp in (http, planner, vision, manager, safety, robot):
        try:
            comp.shutdown()
        except Exception:
            log.exception("shutdown error in %s", type(comp).__name__)


if __name__ == "__main__":
    main()
