"""
HTTP bridge — Layer 5. Endpoint contract is FIXED (the dashboard expects it):

    GET  /api/state       mission, target, battery, zones, last sighting, health
    GET  /api/goals       active (live feedback), pending queue, recent history
    GET  /api/vision      latest vision result
    GET  /camera/stream   MJPEG
    POST /api/target      {"target": "person in a red jacket"}
    POST /api/recall      {}

Every payload carries "timestamp" (unix seconds, server clock) so the client
can compute data age. CORS is wide open (config.CORS_ALLOW_ORIGIN) because the
dashboard is served from a laptop on the same WiFi.

Threading: Flask runs in its own thread via werkzeug's threaded server; every
handler only calls SharedState / ActionManager methods, which take their own
locks. No handler touches rclpy. MJPEG generators sleep with time.sleep
between frames (bounded FPS) and exit when the client disconnects.

Flask (not FastAPI) because it has no async runtime and pulls in far fewer
dependencies on the Pi. werkzeug's dev server is fine for two dashboard clients
on a LAN; swap for waitress if you ever need more.
"""
from __future__ import annotations

import threading
import time
from typing import Any, Optional

from flask import Flask, Response, jsonify, request

from .. import config
from ..actions.manager import ActionManager
from ..logging_setup import get_logger, log_event
from ..robot.interface import RobotInterface
from ..state import SharedState
from . import dashboard_adapter as dash

log = get_logger(__name__)


def create_app(state: SharedState, manager: ActionManager, robot: RobotInterface) -> Flask:
    app = Flask("pearl_bridge")
    stream_clients = {"n": 0}
    stream_lock = threading.Lock()

    # ------------------------------------------------------------------ CORS
    @app.after_request
    def cors(resp: Response) -> Response:
        resp.headers["Access-Control-Allow-Origin"] = config.CORS_ALLOW_ORIGIN
        resp.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
        resp.headers["Access-Control-Allow-Headers"] = "Content-Type"
        resp.headers["Cache-Control"] = "no-store"
        return resp

    @app.route("/api/<path:_any>", methods=["OPTIONS"])
    def preflight(_any: str) -> Response:
        return Response(status=204)

    def stamped(payload: dict[str, Any]) -> Response:
        payload.setdefault("timestamp", time.time())
        return jsonify(payload)

    # ------------------------------------------------------------------ GET
    # These three emit the dashboard's contract (ms timestamps, mapped
    # component keys, single active goal, bbox object). See dashboard_adapter.py.
    @app.get("/api/state")
    def api_state() -> Response:
        return jsonify(dash.state_payload(state, manager))

    @app.get("/api/goals")
    def api_goals() -> Response:
        return jsonify(dash.goals_payload(manager))

    @app.get("/api/vision")
    def api_vision() -> Response:
        return jsonify(dash.vision_payload(state))

    @app.get("/api/health")           # convenience, not part of the fixed contract
    def api_health() -> Response:
        return stamped({"health": state.health(), "safety": state.safety()})

    # Raw internal snapshots, unmapped — handy when debugging the adapter.
    @app.get("/api/debug/state_raw")
    def api_state_raw() -> Response:
        return stamped(state.snapshot_state())

    # ------------------------------------------------------------------ POST
    @app.post("/api/target")
    def api_target() -> Response:
        body = request.get_json(silent=True) or {}
        target: Optional[str] = body.get("target") or body.get("description")
        if not target or not isinstance(target, str) or not target.strip():
            return stamped({"ok": False, "error": "body must be {\"target\": \"<description>\"}"}), 400
        target = target.strip()[:200]
        manager.cancel_all(reason="new_target")
        state.set_target(target)
        log_event(log, "operator.set_target", target=target, remote=request.remote_addr)
        return stamped({"ok": True, "target": target, "mission": state.mission()})

    @app.post("/api/behavior")
    def api_behavior() -> Response:
        body = request.get_json(silent=True) or {}
        mode = body.get("mode")
        if not state.set_behavior(mode):
            return stamped({"ok": False, "error": f"unknown mode: {mode}"}), 400
        log_event(log, "operator.set_behavior", mode=mode, remote=request.remote_addr)
        return stamped({"ok": True, "mode": mode})

    @app.post("/api/recall")
    def api_recall() -> Response:
        goal = manager.recall()
        log_event(log, "operator.recall", remote=request.remote_addr, hold_goal=goal.id)
        return stamped({"ok": True, "goal_id": goal.id, "mission": state.mission(), "hold_goal": goal.to_dict()})

    # ------------------------------------------------------------------ debug (bring-up only)
    if config.DEBUG_ENDPOINTS:
        from ..actions.goal import Goal, SOURCE_OPERATOR

        @app.post("/api/debug/goal")
        def debug_goal() -> Response:
            """Submit a goal by hand, bypassing the planner. Operator source => priority may be 0-99.
            curl -X POST -H 'Content-Type: application/json' \
                 -d '{"action":"navigate_to_zone","params":{"zone_id":"zone_a"},"priority":50}' :8000/api/debug/goal"""
            body = request.get_json(silent=True) or {}
            goal = Goal(action=str(body.get("action", "")), params=body.get("params") or {},
                        priority=int(body.get("priority", 50)), source=SOURCE_OPERATOR,
                        ttl_seconds=body.get("ttl_seconds"))
            manager.submit(goal)
            log_event(log, "operator.debug_goal", goal_id=goal.id, action=goal.action, state=goal.state, reason=goal.reason)
            return stamped({"ok": goal.state not in ("REJECTED",), "goal": goal.to_dict()})

        @app.post("/api/debug/cancel_all")
        def debug_cancel() -> Response:
            manager.cancel_all()
            return stamped({"ok": True})

        @app.get("/api/debug/config")
        def debug_config() -> Response:
            keep = {k: v for k, v in vars(config).items()
                    if k.isupper() and isinstance(v, (str, int, float, bool, list, dict, tuple, type(None)))}
            return stamped({"config": keep})

    # ------------------------------------------------------------------ MJPEG
    @app.get("/camera/stream")
    def camera_stream() -> Response:
        with stream_lock:
            if stream_clients["n"] >= config.MJPEG_MAX_CLIENTS:
                return stamped({"ok": False, "error": "too many stream clients"}), 503
            stream_clients["n"] += 1

        def gen():
            period = 1.0 / config.MJPEG_FPS
            last_ts = -1.0
            try:
                while True:
                    t0 = time.time()
                    jpeg, ts = robot.latest_jpeg(config.MJPEG_WIDTH, config.MJPEG_JPEG_QUALITY)
                    if jpeg is not None and ts != last_ts:
                        last_ts = ts
                        yield (b"--frame\r\nContent-Type: image/jpeg\r\n"
                               + f"Content-Length: {len(jpeg)}\r\nX-Timestamp: {ts:.3f}\r\n\r\n".encode()
                               + jpeg + b"\r\n")
                    time.sleep(max(0.0, period - (time.time() - t0)))
            finally:
                with stream_lock:
                    stream_clients["n"] -= 1

        return Response(gen(), mimetype="multipart/x-mixed-replace; boundary=frame")

    @app.get("/")
    def index() -> Response:
        return stamped({"service": "pearl_backend_v1", "stub_mode": config.STUB_MODE,
                        "endpoints": ["/api/state", "/api/goals", "/api/vision", "/camera/stream",
                                      "POST /api/target", "POST /api/behavior", "POST /api/recall"]})

    return app


class HttpServer:
    """Runs the Flask app in a daemon thread."""

    def __init__(self, app: Flask) -> None:
        self.app = app
        self._thread: Optional[threading.Thread] = None
        self._server = None

    def start(self) -> None:
        from werkzeug.serving import make_server
        self._server = make_server(config.HTTP_HOST, config.HTTP_PORT, self.app, threaded=True)
        self._thread = threading.Thread(target=self._server.serve_forever, name="http", daemon=True)
        self._thread.start()
        log_event(log, "http.started", host=config.HTTP_HOST, port=config.HTTP_PORT)

    def shutdown(self) -> None:
        if self._server:
            self._server.shutdown()
