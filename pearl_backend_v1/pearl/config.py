"""
pearl/config.py — THE single source of truth for every hardware identifier,
port, threshold, interval and tuning constant in the backend.

Nothing else in the project may hard-code a topic name, action name, message
type, port, or magic number. If you find one elsewhere, move it here.

Every value marked  TODO(hw)  is a PLACEHOLDER for something I could not know
without the robot in front of me. Each one has a comment saying how to discover
the real value on the Pi. Search this file for "TODO(hw)" to get the full list.

Environment overrides
---------------------
  PEARL_STUB=1|0      -> STUB_MODE (default 1 = run without robot/ROS2)
  PEARL_LOG_LEVEL     -> DEBUG / INFO / WARNING
  PEARL_HTTP_PORT     -> HTTP bridge port
  PEARL_VLM_URL       -> vision model server URL
  PEARL_PLANNER_URL   -> planner model server URL
"""
from __future__ import annotations

import os

# =============================================================================
# 0. RUN MODE
# =============================================================================

#: Single switch. True  = no ROS2, no robot, no model servers required; every
#: component is replaced by a plausible synthetic equivalent on the SAME
#: interfaces. False = real robot under ROS2 Humble.
STUB_MODE: bool = os.environ.get("PEARL_STUB", "1") == "1"

#: In stub mode, still talk to real llama.cpp servers (lets you develop prompts
#: on a laptop with the synthetic robot). Ignored when STUB_MODE is False.
STUB_USE_REAL_MODELS: bool = os.environ.get("PEARL_STUB_REAL_MODELS", "0") == "1"

#: "ackermann" (MentorPi A1) or "mecanum" (MentorPi M1). Affects whether lateral
#: velocity (vy) is ever commanded. TODO(hw): set to your chassis.
CHASSIS_VARIANT: str = "ackermann"

LOG_LEVEL: str = os.environ.get("PEARL_LOG_LEVEL", "INFO")
LOG_DIR: str = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")
LOG_JSONL_FILE: str = os.path.join(LOG_DIR, "pearl.jsonl")
LOG_JSONL_MAX_BYTES: int = 20 * 1024 * 1024
LOG_JSONL_BACKUPS: int = 3

# =============================================================================
# 1. ROS2 IDENTIFIERS  — all TODO(hw)
# =============================================================================
# Discovery on the robot (with the Hiwonder stock stack running):
#   ros2 topic list -t                # topics with types
#   ros2 topic info -v /some/topic    # publishers, QoS (important for lidar/camera)
#   ros2 action list -t               # action names with types
#   ros2 interface show <type>        # field layout of a message/action
#   ros2 run tf2_tools view_frames    # frame names (map / odom / base_*)

ROS_NODE_NAME: str = "pearl_backend_v1"

#: Topic names. Values below are educated guesses at Hiwonder's naming and are
#: UNVERIFIED — treat every one as TODO(hw) and confirm with `ros2 topic list -t`.
TOPICS: dict[str, str] = {
    # Velocity command that actually reaches the motor controller.
    # TODO(hw): Hiwonder often uses /controller/cmd_vel; plain /cmd_vel is also common.
    "cmd_vel_motor": "/controller/cmd_vel",
    # Velocity output of Nav2's controller server. If SAFETY_GATE_CMD_VEL is
    # True, remap Nav2 to publish HERE (not to cmd_vel_motor) so the safety
    # floor sits between Nav2 and the motors. See safety/safety_floor.py.
    # TODO(hw): set via Nav2 params (controller_server ... cmd_vel topic remap).
    "cmd_vel_nav": "/cmd_vel_nav",
    "lidar_scan": "/scan",                                   # TODO(hw)
    "camera_image": "/ascamera/camera_publisher/rgb0/image", # TODO(hw) raw image
    "camera_image_compressed": "/ascamera/camera_publisher/rgb0/image/compressed",  # TODO(hw)
    "battery": "/ros_robot_controller/battery",              # TODO(hw)
    "odom": "/odom",                                         # TODO(hw)
    # Pan-tilt servo command topic. Completely unknown. TODO(hw)
    "pan_tilt_servo": "/ros_robot_controller/bus_servo/set_position",
}

#: Fully-qualified message types, resolved at runtime with
#: rosidl_runtime_py.utilities.get_message so you only edit strings here.
MSG_TYPES: dict[str, str] = {
    "cmd_vel": "geometry_msgs/msg/Twist",
    "lidar_scan": "sensor_msgs/msg/LaserScan",
    "camera_image": "sensor_msgs/msg/Image",
    "camera_image_compressed": "sensor_msgs/msg/CompressedImage",
    # TODO(hw): Hiwonder's battery topic may be std_msgs/UInt16 (millivolts),
    # sensor_msgs/BatteryState, or a custom ros_robot_controller_msgs type.
    "battery": "sensor_msgs/msg/BatteryState",
    "odom": "nav_msgs/msg/Odometry",
    # TODO(hw): probably ros_robot_controller_msgs/msg/SetBusServoState or similar.
    "pan_tilt_servo": "std_msgs/msg/String",
}

#: Use the compressed image topic (much cheaper: bytes stay JPEG until needed).
#: TODO(hw): check `ros2 topic list | grep compressed`.
CAMERA_USE_COMPRESSED: bool = True

#: How to turn a battery message into (percent 0-100, volts). Edit this one
#: function when you know the real message type. TODO(hw)
def battery_msg_to_percent_volts(msg) -> tuple[float | None, float | None]:
    volts = getattr(msg, "voltage", None)
    pct = getattr(msg, "percentage", None)
    if pct is not None and 0.0 <= pct <= 1.0:
        pct = pct * 100.0
    if pct is None and volts is not None:
        pct = voltage_to_percent(volts)
    if volts is None and hasattr(msg, "data"):        # e.g. UInt16 millivolts
        volts = float(msg.data) / 1000.0
        pct = voltage_to_percent(volts)
    return pct, volts

#: Linear LiPo approximation used when the message has no percentage.
#: TODO(hw): MentorPi ships a 2S or 3S pack — set the cell count.
BATTERY_CELLS: int = 3
BATTERY_CELL_EMPTY_V: float = 3.5
BATTERY_CELL_FULL_V: float = 4.2

def voltage_to_percent(volts: float) -> float:
    lo = BATTERY_CELLS * BATTERY_CELL_EMPTY_V
    hi = BATTERY_CELLS * BATTERY_CELL_FULL_V
    return max(0.0, min(100.0, (volts - lo) / (hi - lo) * 100.0))

#: Nav2 action. These are the standard Nav2 names and are *probably* right,
#: but confirm with `ros2 action list -t`. TODO(hw)
NAV2_ACTION_NAME: str = "/navigate_to_pose"
NAV2_ACTION_TYPE: str = "nav2_msgs/action/NavigateToPose"

#: Frames. TODO(hw): `ros2 run tf2_tools view_frames` — base_footprint vs base_link.
FRAME_MAP: str = "map"
FRAME_BASE: str = "base_footprint"

#: Seconds to wait for the Nav2 action server before marking nav2 health "down".
NAV2_SERVER_WAIT_S: float = 2.0

#: How the robot's map-frame pose is obtained: "tf" (map->base via tf2, correct
#: when SLAM/AMCL is running) or "odom" (fallback, drifts). TODO(hw)
POSE_SOURCE: str = "tf"
POSE_POLL_HZ: float = 2.0

# =============================================================================
# 2. MODEL SERVERS (llama.cpp, OpenAI-compatible)
# =============================================================================
#: Vision-language model: Liquid AI LFM2.5-VL-450M (quantized GGUF).
VLM_URL: str = os.environ.get("PEARL_VLM_URL", "http://127.0.0.1:8080/v1/chat/completions")
VLM_MODEL_NAME: str = "lfm2.5-vl-450m"          # only echoed in the request body
VLM_TIMEOUT_S: float = 6.0
VLM_MAX_TOKENS: int = 96
VLM_TEMPERATURE: float = 0.0

#: Planner model: Liquid AI LFM2.5-230M (quantized GGUF). NOT a reasoning model.
PLANNER_URL: str = os.environ.get("PEARL_PLANNER_URL", "http://127.0.0.1:8081/v1/chat/completions")
PLANNER_MODEL_NAME: str = "lfm2.5-230m"
PLANNER_TIMEOUT_S: float = 4.0
PLANNER_MAX_TOKENS: int = 160
PLANNER_TEMPERATURE: float = 0.0

#: Health probe endpoint suffix (llama.cpp serves GET /health).
MODEL_HEALTH_PATH: str = "/health"
MODEL_HEALTH_INTERVAL_S: float = 5.0

# =============================================================================
# 3. VISION (Layer 1)
# =============================================================================
VISION_VLM_INTERVAL_S: float = 1.5      #: identity verification cadence (NOT every frame)
VISION_TRACK_INTERVAL_S: float = 0.25   #: cheap classical tracker cadence
VISION_VLM_INPUT_WIDTH: int = 320       #: downscale before base64 -> VLM
VISION_VLM_JPEG_QUALITY: int = 70
VISION_MATCH_CONFIDENCE_MIN: float = 0.6  #: below this a "match" is treated as no match
VISION_SIGHTING_STALE_S: float = 20.0     #: last sighting older than this = "lost"
VISION_TRACKER_MAX_MISSES: int = 8        #: consecutive tracker failures before drop
VISION_TRACKER_SCORE_MIN: float = 0.55    #: template-match score threshold
VISION_TRACKER_WORK_WIDTH: int = 160      #: tracker runs on this width (grayscale)
#: Horizontal field of view of the RGB camera, used to convert bbox-x to bearing.
#: TODO(hw): check the depth camera datasheet (ascamera/Astra-style ≈ 60-70°).
CAMERA_HFOV_DEG: float = 65.0

# =============================================================================
# 4. PLANNER (Layer 2)
# =============================================================================
PLANNER_INTERVAL_S: float = 2.0
PLANNER_PROMPT_VERSION: str = "v1"       #: key into planner/prompts.py PROMPT_VERSIONS
#: "json_schema" -> llama.cpp converts our schema to a grammar server-side (preferred)
#: "gbnf"        -> send planner/grammar.gbnf as `grammar`
#: "native"      -> OpenAI `tools` param (needs llama-server --jinja + template support)
#: "none"        -> free text, rely on parser fallback only
PLANNER_CONSTRAIN_MODE: str = "json_schema"
PLANNER_GBNF_PATH: str = os.path.join(os.path.dirname(os.path.abspath(__file__)), "planner", "grammar.gbnf")
PLANNER_MAX_CALLS_PER_CYCLE: int = 2
PLANNER_RECENT_OUTCOMES_N: int = 5
#: Safe default when the planner fails (timeout / dead server / invalid output).
#: Applied only if no goal is executing; otherwise the current goal continues.
PLANNER_FALLBACK_ACTION: str = "hold_position"
PLANNER_FALLBACK_PARAMS: dict = {"seconds": 3.0}
PLANNER_FALLBACK_PRIORITY: int = 10

# =============================================================================
# 5. ACTION LAYER (Layer 3)
# =============================================================================
#: Priority bands. Planner may only use PRIORITY_PLANNER_MIN..MAX. Operator and
#: system goals (recall) live above it so they always win preemption.
PRIORITY_PLANNER_MIN: int = 0
PRIORITY_PLANNER_MAX: int = 79
PRIORITY_RECALL: int = 95

ACTION_TICK_HZ: float = 5.0             #: manager tick (TTL checks, executor stepping)
GOAL_QUEUE_MAX: int = 8                 #: pending goals per resource beyond which -> reject queue_full
GOAL_REGISTRY_MAX: int = 200            #: >= 50 required by dashboard
GOAL_HISTORY_API_N: int = 50            #: how many terminal goals /api/goals returns

#: Default TTL per action (seconds since the goal's triggering evidence).
#: None = no TTL.
GOAL_DEFAULT_TTL_S: dict[str, float | None] = {
    "navigate_to_zone": 120.0,
    "explore_until_found": None,        # explore carries its own ttl_seconds param
    "approach_target": 45.0,            # sighting-driven; goes stale fast
    "reposition_for_concealment": 60.0,
    "hold_position": None,
    "scan_with_camera": 15.0,
}

#: Resources (mutexes). Chassis and camera pan-tilt are independent.
RESOURCE_CHASSIS: str = "chassis"
RESOURCE_PAN_TILT: str = "pan_tilt"

#: Below this percent, motion goals are rejected with reason insufficient_battery.
BATTERY_MIN_PERCENT_FOR_MOTION: float = 15.0

#: Hold position default duration if planner omits it.
HOLD_DEFAULT_SECONDS: float = 5.0
#: Approach: re-issue the Nav2 goal at most this often while target visible.
APPROACH_REPLAN_INTERVAL_S: float = 3.0
APPROACH_DEFAULT_STANDOFF_M: float = 1.5
#: Assumed target range when lidar has no return at the bearing.
APPROACH_FALLBACK_RANGE_M: float = 3.0
#: Zone is "reached" when Nav2 reports distance_remaining below this.
ZONE_REACHED_DISTANCE_M: float = 0.5
#: Pan-tilt settle time after a scan_with_camera command.
PAN_TILT_SETTLE_S: float = 1.0
PAN_LIMIT_DEG: tuple[float, float] = (-90.0, 90.0)
TILT_LIMIT_DEG: tuple[float, float] = (-30.0, 45.0)

# =============================================================================
# 6. MAP GEOMETRY  — zones, geofence, concealment points  (map frame, metres)
# =============================================================================
# TODO(hw): replace with coordinates from YOUR SLAM map. Get them by driving the
# robot around with `ros2 topic echo /amcl_pose` or `ros2 run tf2_ros tf2_echo map base_footprint`
# and reading off (x, y) at the spots you care about.

#: Named exploration zones: id -> (x, y, yaw_rad). Planner refers to these by id.
ZONES: dict[str, tuple[float, float, float]] = {
    "zone_a": (1.0, 0.0, 0.0),        # TODO(hw)
    "zone_b": (2.0, 1.5, 1.57),       # TODO(hw)
    "zone_c": (0.0, 2.5, 3.14),       # TODO(hw)
    "zone_d": (-1.5, 1.0, -1.57),     # TODO(hw)
}

#: Geofence polygon (list of (x, y)), convex or not. Goals whose target pose
#: falls outside are REJECTED before any motion; the safety floor also stops the
#: chassis if the robot itself leaves it. TODO(hw)
GEOFENCE_POLYGON: list[tuple[float, float]] = [
    (-3.0, -2.0), (4.0, -2.0), (4.0, 4.0), (-3.0, 4.0),
]
#: Safety floor triggers when the robot is within this distance of the fence
#: edge (moving fast toward a fence you can't see is the failure mode here).
GEOFENCE_MARGIN_M: float = 0.25

#: Candidate concealment poses (x, y, yaw). v1 picks the nearest. TODO(hw)
CONCEALMENT_POINTS: list[tuple[float, float, float]] = [
    (-1.0, -1.0, 0.0),
    (3.0, 3.0, 3.14),
]

#: Home / recall pose. None = recall just holds position. TODO(hw)
HOME_POSE: tuple[float, float, float] | None = None

# =============================================================================
# 7. SAFETY FLOOR (Layer 4)
# =============================================================================
#: Forward arc considered for collision stop, degrees either side of +x.
LIDAR_STOP_ARC_DEG: float = 30.0
#: Stop if any valid return in the arc is closer than this.
LIDAR_STOP_DISTANCE_M: float = 0.30
#: Resume only when clear beyond this (hysteresis).
LIDAR_RESUME_DISTANCE_M: float = 0.45
#: Ignore returns below this (self-hits / noise). TODO(hw): check scan range_min.
LIDAR_MIN_VALID_M: float = 0.05
#: If no scan arrives for this long the floor declares lidar "down" and STOPS
#: (fail closed — a blind robot does not drive).
LIDAR_TIMEOUT_S: float = 1.0
#: While stopped, republish zero velocity at this rate.
SAFETY_STOP_PUBLISH_HZ: float = 10.0
#: True: the floor gates Nav2's cmd_vel (subscribe cmd_vel_nav, republish to
#: cmd_vel_motor only when safe). This is the *correct* wiring. False: the floor
#: just publishes zeros to cmd_vel_motor and fights Nav2 (works, ugly). TODO(hw)
SAFETY_GATE_CMD_VEL: bool = True
#: The lidar's mounting yaw relative to base frame (deg). 0 if the scan's
#: angle 0 points forward. TODO(hw): many lidars are mounted backwards (180).
LIDAR_YAW_OFFSET_DEG: float = 0.0

# =============================================================================
# 8. HTTP BRIDGE (Layer 5)
# =============================================================================
HTTP_HOST: str = "0.0.0.0"
HTTP_PORT: int = int(os.environ.get("PEARL_HTTP_PORT", "8000"))
CORS_ALLOW_ORIGIN: str = "*"
MJPEG_FPS: float = 6.0                  #: cap; 5-10 is all a loaded Pi can spare
MJPEG_WIDTH: int = 320                  #: downscale for the stream
MJPEG_JPEG_QUALITY: int = 60
MJPEG_MAX_CLIENTS: int = 2
#: Bring-up helpers: POST /api/debug/goal, POST /api/debug/cancel_all, GET /api/debug/config.
#: Turn off once the planner is trusted; anyone on the WiFi can drive the robot with these.
DEBUG_ENDPOINTS: bool = os.environ.get("PEARL_DEBUG", "1") == "1"

# =============================================================================
# 9. STUB / SIMULATION
# =============================================================================
STUB_TICK_HZ: float = 10.0
STUB_ROBOT_SPEED_MPS: float = 0.35
STUB_BATTERY_DRAIN_PCT_PER_MIN: float = 0.5
STUB_TARGET_POSE: tuple[float, float] = (2.0, 1.5)   #: synthetic target sits near zone_b
STUB_TARGET_VISIBLE_RANGE_M: float = 2.0
STUB_OBSTACLE_PROBABILITY: float = 0.004             #: per tick, random lidar close return
STUB_FRAME_SIZE: tuple[int, int] = (320, 240)        #: (w, h)
STUB_VLM_LATENCY_S: float = 0.4
STUB_PLANNER_LATENCY_S: float = 0.2
