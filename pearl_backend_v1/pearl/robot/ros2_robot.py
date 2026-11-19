"""
Ros2Robot — the real robot under ROS2 Humble (rclpy).

This module is imported ONLY when config.STUB_MODE is False, so the rest of
the project never needs ROS2 installed. It has not been run against real
hardware yet: every identifier comes from config.py and is marked TODO(hw)
there. Read the "Threading" note before changing anything.

Threading
  * rclpy spins in ITS OWN thread (MultiThreadedExecutor, 2 threads) started
    by start(). Subscription callbacks run there and only write small values
    under self._lock or hand off to registered callbacks (safety floor).
  * Everything else (action tick, planner, vision, Flask) lives in other
    threads and reads through the getters. Nothing in those threads calls
    rclpy spin functions.
  * Nav2 action-client futures complete on the executor thread; we translate
    them into plain callbacks the action layer expects.

Message types are resolved from strings in config.MSG_TYPES so this file does
not need editing when you discover the real types.
"""
from __future__ import annotations

import math
import threading
import time
from typing import Any, Callable, Optional

import cv2
import numpy as np

from .. import config
from ..geometry import yaw_from_quaternion, quaternion_from_yaw
from ..logging_setup import get_logger, log_event
from ..state import Pose2D, SharedState
from .interface import (RobotInterface, LidarSummary, NavFeedback, NavHandle,
                        NAV_SUCCEEDED, NAV_ABORTED, NAV_CANCELED, NAV_REJECTED, NAV_SERVER_DOWN)

log = get_logger(__name__)


class Ros2Robot(RobotInterface):
    def __init__(self, state: SharedState) -> None:
        self.state = state
        self._lock = threading.RLock()
        self._lidar_cbs: list[Callable[[LidarSummary], None]] = []
        self._gate_open = True
        self._last_scan: Optional[LidarSummary] = None
        self._pose: Optional[Pose2D] = None
        self._img_msg: Any = None
        self._img_ts = 0.0
        self._decoded: tuple[Any, Optional[np.ndarray]] = (None, None)   # (msg identity, frame)
        self._battery_ts = 0.0
        self._nav_available = False
        self._exec_thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------ lifecycle
    def start(self) -> None:
        import rclpy
        from rclpy.executors import MultiThreadedExecutor
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        from rclpy.action import ActionClient
        from rosidl_runtime_py.utilities import get_message, get_action

        rclpy.init()
        self.node = rclpy.create_node(config.ROS_NODE_NAME)
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)

        T, M = config.TOPICS, config.MSG_TYPES
        self._Twist = get_message(M["cmd_vel"])
        self.node.create_subscription(get_message(M["lidar_scan"]), T["lidar_scan"], self._on_scan, sensor_qos)
        if config.CAMERA_USE_COMPRESSED:
            self.node.create_subscription(get_message(M["camera_image_compressed"]), T["camera_image_compressed"],
                                          self._on_image, sensor_qos)
        else:
            self.node.create_subscription(get_message(M["camera_image"]), T["camera_image"], self._on_image, sensor_qos)
        self.node.create_subscription(get_message(M["battery"]), T["battery"], self._on_battery, 10)
        self.node.create_subscription(get_message(M["odom"]), T["odom"], self._on_odom, sensor_qos)

        self._cmd_pub = self.node.create_publisher(self._Twist, T["cmd_vel_motor"], 10)
        if config.SAFETY_GATE_CMD_VEL:
            # Relay Nav2's velocity to the motors only while the gate is open.
            self.node.create_subscription(self._Twist, T["cmd_vel_nav"], self._on_cmd_vel_nav, 10)

        # Pan-tilt: TODO(hw) — type and topic unknown; publisher created only if configured.
        try:
            self._servo_pub = self.node.create_publisher(get_message(M["pan_tilt_servo"]), T["pan_tilt_servo"], 10)
        except Exception as e:  # unknown package etc.
            log.warning("pan-tilt publisher not created (%s); scan_with_camera will be a no-op", e)
            self._servo_pub = None

        # TF for map-frame pose
        self._tf_buffer = None
        if config.POSE_SOURCE == "tf":
            try:
                from tf2_ros import Buffer, TransformListener
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self.node, spin_thread=False)
                self.node.create_timer(1.0 / config.POSE_POLL_HZ, self._poll_tf)
            except Exception as e:
                log.warning("tf2 unavailable (%s); falling back to odom pose", e)

        # Nav2 action client
        self._NavAction = get_action(config.NAV2_ACTION_TYPE)
        self._nav_client = ActionClient(self.node, self._NavAction, config.NAV2_ACTION_NAME)
        self.node.create_timer(2.0, self._health_timer)

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self.node)
        self._exec_thread = threading.Thread(target=self._executor.spin, name="ros-executor", daemon=True)
        self._exec_thread.start()
        self.state.set_health("ros", "ok")
        log_event(log, "ros.started", node=config.ROS_NODE_NAME, topics=config.TOPICS)

    def shutdown(self) -> None:
        import rclpy
        try:
            self._executor.shutdown(timeout_sec=1.0)
            self.node.destroy_node()
            rclpy.shutdown()
        except Exception:
            log.exception("ros shutdown error")

    # ------------------------------------------------------------------ callbacks (executor thread)
    def _on_scan(self, msg: Any) -> None:
        now = time.time()
        offset = math.radians(config.LIDAR_YAW_OFFSET_DEG)
        arc = math.radians(config.LIDAR_STOP_ARC_DEG)
        min_fwd: Optional[float] = None
        bins: dict[int, float] = {}
        ang = msg.angle_min
        inc = msg.angle_increment
        rmin = max(config.LIDAR_MIN_VALID_M, msg.range_min)
        for r in msg.ranges:
            a = ang + offset
            ang += inc
            if not (rmin <= r <= msg.range_max) or r != r:      # NaN check
                continue
            a = math.atan2(math.sin(a), math.cos(a))
            if abs(a) <= arc and (min_fwd is None or r < min_fwd):
                min_fwd = r
            d = int(round(math.degrees(a)))
            if d not in bins or r < bins[d]:
                bins[d] = r
        scan = LidarSummary(ts=now, min_forward=min_fwd, ranges_by_deg=bins)
        with self._lock:
            self._last_scan = scan
        for cb in self._lidar_cbs:
            try:
                cb(scan)
            except Exception:
                log.exception("lidar callback failed")

    def _on_image(self, msg: Any) -> None:
        with self._lock:
            self._img_msg = msg          # reference only; decode lazily
            self._img_ts = time.time()

    def _on_battery(self, msg: Any) -> None:
        pct, volts = config.battery_msg_to_percent_volts(msg)
        self._battery_ts = time.time()
        self.state.set_battery(pct, volts)

    def _on_odom(self, msg: Any) -> None:
        if config.POSE_SOURCE == "tf" and self._tf_buffer is not None:
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        pose = Pose2D(p.x, p.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
        with self._lock:
            self._pose = pose
        self.state.set_pose(pose)

    def _poll_tf(self) -> None:
        try:
            from rclpy.time import Time
            tf = self._tf_buffer.lookup_transform(config.FRAME_MAP, config.FRAME_BASE, Time())
            t, q = tf.transform.translation, tf.transform.rotation
            pose = Pose2D(t.x, t.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))
            with self._lock:
                self._pose = pose
            self.state.set_pose(pose)
        except Exception as e:      # tf2 raises several exception types
            log.debug("tf lookup failed: %s", e)

    def _on_cmd_vel_nav(self, msg: Any) -> None:
        if self._gate_open:
            self._cmd_pub.publish(msg)
        # gate closed: the safety floor is publishing zeros; Nav2's command is dropped.

    def _health_timer(self) -> None:
        now = time.time()
        self.state.set_health("ros", "ok")     # executor is spinning if this timer fires
        self.state.set_health("camera", "ok" if now - self._img_ts < 2.0 else "down")
        scan = self._last_scan
        self.state.set_health("lidar", "ok" if scan and now - scan.ts < config.LIDAR_TIMEOUT_S else "down")
        self.state.set_health("battery", "ok" if now - self._battery_ts < 10.0 else "down")
        self._nav_available = self._nav_client.server_is_ready()
        self.state.set_health("nav2", "ok" if self._nav_available else "down")

    # ------------------------------------------------------------------ sensing getters (any thread)
    def latest_frame(self) -> tuple[Optional[np.ndarray], float]:
        with self._lock:
            msg, ts = self._img_msg, self._img_ts
            ident, frame = self._decoded
        if msg is None:
            return None, ts
        if ident is msg and frame is not None:
            return frame, ts
        frame = self._decode(msg)
        with self._lock:
            self._decoded = (msg, frame)
        return frame, ts

    @staticmethod
    def _decode(msg: Any) -> Optional[np.ndarray]:
        try:
            if hasattr(msg, "format"):                               # CompressedImage
                return cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
            enc = msg.encoding.lower()                               # raw Image, no cv_bridge needed
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            if enc in ("bgr8", "rgb8"):
                img = buf.reshape(msg.height, msg.width, 3)
                return img if enc == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
            if enc == "mono8":
                return cv2.cvtColor(buf.reshape(msg.height, msg.width), cv2.COLOR_GRAY2BGR)
            # TODO(hw): add the camera's actual encoding here (yuv422? bgra8?), or use cv_bridge.
            log.warning("unsupported image encoding %s", enc)
            return None
        except Exception:
            log.exception("image decode failed")
            return None

    def latest_jpeg(self, max_width: int, quality: int) -> tuple[Optional[bytes], float]:
        frame, ts = self.latest_frame()
        if frame is None:
            return None, ts
        if frame.shape[1] > max_width:
            scale = max_width / frame.shape[1]
            frame = cv2.resize(frame, (max_width, int(frame.shape[0] * scale)), interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, quality])
        return (buf.tobytes() if ok else None), ts

    def pose(self) -> Optional[Pose2D]:
        with self._lock:
            return self._pose

    def lidar(self) -> Optional[LidarSummary]:
        with self._lock:
            return self._last_scan

    def register_lidar_callback(self, cb: Callable[[LidarSummary], None]) -> None:
        self._lidar_cbs.append(cb)

    # ------------------------------------------------------------------ motion
    def send_nav_goal(self, x, y, yaw, on_feedback, on_result) -> NavHandle:
        handle = NavHandle(f"nav-{int(time.time() * 1000) % 100000}")
        if not self._nav_client.wait_for_server(timeout_sec=config.NAV2_SERVER_WAIT_S):
            handle.done, handle.result = True, NAV_SERVER_DOWN
            self.state.set_health("nav2", "down", "server not ready")
            on_result(NAV_SERVER_DOWN)
            return handle

        goal = self._NavAction.Goal()
        goal.pose.header.frame_id = config.FRAME_MAP
        goal.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.x, goal.pose.pose.orientation.y = qx, qy
        goal.pose.pose.orientation.z, goal.pose.pose.orientation.w = qz, qw

        def fb_cb(fb_msg: Any) -> None:
            f = fb_msg.feedback
            on_feedback(NavFeedback(
                distance_remaining=getattr(f, "distance_remaining", None),
                navigation_time_s=getattr(getattr(f, "navigation_time", None), "sec", None),
                recoveries=getattr(f, "number_of_recoveries", 0)))

        def goal_response(fut: Any) -> None:
            gh = fut.result()
            if not gh.accepted:
                handle.done, handle.result = True, NAV_REJECTED
                on_result(NAV_REJECTED)
                return
            handle.raw = gh
            gh.get_result_async().add_done_callback(result_cb)

        def result_cb(fut: Any) -> None:
            from action_msgs.msg import GoalStatus
            status = fut.result().status
            res = {GoalStatus.STATUS_SUCCEEDED: NAV_SUCCEEDED, GoalStatus.STATUS_CANCELED: NAV_CANCELED,
                   GoalStatus.STATUS_ABORTED: NAV_ABORTED}.get(status, NAV_ABORTED)
            handle.done, handle.result = True, res
            on_result(res)

        self._nav_client.send_goal_async(goal, feedback_callback=fb_cb).add_done_callback(goal_response)
        return handle

    def cancel_nav_goal(self, handle: NavHandle) -> None:
        gh = handle.raw
        if gh is not None and not handle.done:
            try:
                gh.cancel_goal_async()
            except Exception:
                log.exception("nav cancel failed")

    def publish_cmd_vel(self, vx: float, vy: float, wz: float) -> None:
        msg = self._Twist()
        msg.linear.x = float(vx)
        msg.linear.y = float(vy) if config.CHASSIS_VARIANT == "mecanum" else 0.0
        msg.angular.z = float(wz)
        self._cmd_pub.publish(msg)

    def emergency_stop_publish(self) -> None:
        self._cmd_pub.publish(self._Twist())        # all zeros, straight to the motor topic

    def set_cmd_vel_gate(self, allow: bool) -> None:
        self._gate_open = allow

    def set_pan_tilt(self, pan_deg: float, tilt_deg: float) -> None:
        # TODO(hw): build the real servo message (ids, positions, duration) once the
        # type in config.MSG_TYPES["pan_tilt_servo"] is known. Until then: log only.
        log.warning("set_pan_tilt(%.0f, %.0f) — TODO(hw): servo message not implemented", pan_deg, tilt_deg)

    def nav2_available(self) -> bool:
        return self._nav_available
