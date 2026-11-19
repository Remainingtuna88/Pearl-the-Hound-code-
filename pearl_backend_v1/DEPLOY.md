# Putting the backend on the car — bring-up guide

Follow this top to bottom. Each phase has a "you are done when" check. Do not skip
ahead to wheels-down; the phases are ordered so that every unknown is resolved before
the robot can move under this software.

Conventions: `laptop$` = your machine, `pi$` = shell on the Raspberry Pi 5. Anything in
`config.py` is `pearl/config.py`. Assumes the project lives at `~/pearl_backend_v1` on
the Pi.

**Placeholders used below:** `<PI_IP>` = your robot's LAN IP (`hostname -I` on the Pi);
`<hiwonder_ws>` = the vendor ROS2 workspace path; `<hiwonder-stack>` = the vendor's stock
ROS2 systemd service name. Fill them in when you run the commands — don't commit them.

---

## Phase 0 — On the laptop, before touching the robot (30 min)

1. Unzip, install, run the tests, run stub mode once, and click through
   `/api/state`, `/api/goals`, `/camera/stream`. You want to know what "healthy" looks
   like before you see it broken on the Pi.
2. Read `config.py` end to end. Every `TODO(hw)` is a question you'll answer in Phase 3.
3. Decide two things now and write them down:
   - **Chassis**: A1 (ackermann) or M1 (mecanum) → `CHASSIS_VARIANT`.
   - **Pi RAM**: 4 GB or 8 GB. With 4 GB, plan on Q4 quants and `-c 1024` for the planner.
4. Make a recall alias you can hit without thinking. On the laptop:
   ```bash
   echo 'alias RECALL="curl -s -X POST http://<PI_IP>:8000/api/recall"' >> ~/.bashrc
   ```
   Fill in the IP in Phase 1. You will use this.

**Done when:** tests pass, stub mode runs, you know your chassis and RAM.

---

## Phase 1 — Get onto the Pi and find out how ROS2 is installed (30–60 min)

This is the phase most likely to surprise you. Hiwonder ships MentorPi on Raspberry Pi
OS, and ROS2 Humble officially targets Ubuntu 22.04, so on their Pi 5 images ROS2 usually
runs **inside a Docker container**, not natively. The backend must run in the same
environment as the rest of the ROS graph (same Python that has `rclpy`, same DDS).

```bash
laptop$ ssh pi@<PI_IP>                    # default creds are in the Hiwonder docs
pi$ cat /etc/os-release | head -2
pi$ which ros2 && echo NATIVE || echo "not on host PATH"
pi$ docker ps                              # if this lists a container with ROS in it: DOCKER
pi$ ps aux | grep -E 'nav2|slam|ros2' | head
```

Then figure out *where* the stock stack lives and how it is launched:

```bash
pi$ ls ~ ; ls ~/ros2_ws ~/*ws* 2>/dev/null
pi$ systemctl list-units --type=service | grep -iE 'ros|hiwonder|start'
pi$ crontab -l; cat ~/.bashrc | grep -iE 'ros|source|docker'
```

Write down:
- **NATIVE or DOCKER.** If Docker: the container name, and how to get a shell in it
  (`docker exec -it <name> bash`). Confirm `python3 -c "import rclpy"` works *inside* it.
- The path of the Hiwonder workspace and its `setup.bash`.
- The launch file / service that starts SLAM, Nav2, camera, YOLO. You will need to
  (a) disable YOLO and (b) remap Nav2's cmd_vel later.
- `echo $ROS_DOMAIN_ID` and `echo $RMW_IMPLEMENTATION` in the environment where ROS runs.

If DOCKER: you will run the backend inside that container. Bind-mount the project so you
can edit on the host: check how the container is started (`docker inspect <name> | grep -A5
Binds`) — often `/home/pi` is already mounted. If not, you'll add
`-v /home/pi/pearl_backend_v1:/root/pearl_backend_v1` to its run command.

**Done when:** you have a shell where `python3 -c "import rclpy; print('ok')"` prints ok
and `ros2 topic list` shows the robot's topics.

---

## Phase 2 — Copy the project and install dependencies (20 min)

```bash
laptop$ scp pearl_backend_v1.zip pi@<PI_IP>:~
pi$ unzip pearl_backend_v1.zip && cd pearl_backend_v1
```

In the ROS environment (host or container — whichever Phase 1 found):

```bash
pi$ python3 --version                       # 3.10+ required
pi$ python3 -c "import numpy, cv2; print(numpy.__version__, cv2.__version__)"   # usually present via ROS
pi$ pip install flask requests --break-system-packages     # drop the flag on Ubuntu 22.04 / in Docker
pi$ python3 -c "import flask, requests; print('ok')"
pi$ PEARL_STUB=1 python3 -m pytest -q                   # all tests should pass, even on the Pi
pi$ PEARL_STUB=1 timeout 20 python3 run.py &            # stub mode ON THE PI — proves the Python side
pi$ curl -s localhost:8000/api/state | head -c 300
```

Do **not** `pip install opencv-python-headless` if `import cv2` already works — it will
shadow the ROS one and can break `cv_bridge` for the stock stack.

**Done when:** tests pass on the Pi and stub mode answers on port 8000.

---

## Phase 3 — Discover the hardware identifiers and fill in config.py (1–2 h)

With the stock Hiwonder stack running (robot on a stand, wheels off the ground):

```bash
pi$ source <hiwonder_ws>/install/setup.bash
pi$ bash scripts/discover_hw.sh | tee ~/hw_discovery.txt
```

Now go through `config.py` with `hw_discovery.txt` open. Checklist:

| config key | What to look for in the discovery output |
|---|---|
| `CHASSIS_VARIANT` | you already know |
| `TOPICS["cmd_vel_motor"]` | the Twist topic the **motor controller node subscribes to** (`ros2 topic info -v` → Subscription count ≥1 from a controller/driver node). Candidates: `/controller/cmd_vel`, `/cmd_vel`. |
| `TOPICS["lidar_scan"]` | the `sensor_msgs/msg/LaserScan` topic. Note its QoS (Reliability: BEST_EFFORT is typical). |
| `LIDAR_YAW_OFFSET_DEG` | `ros2 topic echo --once /scan --field angle_min`. Stand in front of the lidar, echo `/scan`, find which index sees you. If it's near index 0 → offset 0; near the middle → 180. |
| `LIDAR_MIN_VALID_M` | ≥ `range_min` from the scan. |
| `TOPICS["camera_image_compressed"]` | any `sensor_msgs/msg/CompressedImage` under the camera namespace. If none exists set `CAMERA_USE_COMPRESSED=False` and use the raw `Image` topic. |
| `TOPICS["battery"]`, `MSG_TYPES["battery"]` | the "battery-ish" section. Echo one message; adapt `battery_msg_to_percent_volts()` to its fields. Set `BATTERY_CELLS` (look at the pack: 2S ≈ 7.4 V nominal, 3S ≈ 11.1 V). |
| `TOPICS["odom"]` | the `nav_msgs/msg/Odometry` topic. |
| `FRAME_MAP`, `FRAME_BASE`, `POSE_SOURCE` | if `tf2_echo map base_footprint` prints transforms → keep `tf`/`base_footprint`. If it errors but `base_link` works → change `FRAME_BASE`. If neither (no SLAM running) → `POSE_SOURCE="odom"` temporarily. |
| `NAV2_ACTION_NAME/TYPE` | `ros2 action list -t` — confirm `/navigate_to_pose [nav2_msgs/action/NavigateToPose]`. |
| `TOPICS["pan_tilt_servo"]`, `MSG_TYPES` | the "servo-ish" section; then `ros2 interface show <type>`. Implement `Ros2Robot.set_pan_tilt()` in `robot/ros2_robot.py` once you know the fields. **Can stay a no-op for the first missions** — `scan_with_camera` will just log a warning. |
| `CAMERA_HFOV_DEG` | camera datasheet; or measure: put an object at the left edge of frame, note its lidar bearing. |
| `ZONES`, `GEOFENCE_POLYGON`, `CONCEALMENT_POINTS` | **leave for Phase 5** — you need a map first. |

Then sanity-check that Python can resolve every type you wrote:

```bash
pi$ python3 - <<'EOF'
from rosidl_runtime_py.utilities import get_message, get_action
from pearl import config
for k, t in config.MSG_TYPES.items(): get_message(t); print("ok", k, t)
get_action(config.NAV2_ACTION_TYPE); print("ok nav2")
EOF
```

**Done when:** that script prints `ok` for every line and you have no `TODO(hw)` left
unresolved except zones/geofence and (optionally) pan-tilt.

---

## Phase 4 — Build llama.cpp, get the models, benchmark them alone (1–2 h, mostly waiting)

Do this with the stock ROS stack **stopped** so you get clean numbers.

```bash
pi$ sudo apt install -y build-essential cmake git libcurl4-openssl-dev
pi$ git clone https://github.com/ggerganov/llama.cpp ~/llama.cpp && cd ~/llama.cpp
pi$ cmake -B build -DGGML_NATIVE=ON -DLLAMA_CURL=ON
pi$ cmake --build build --config Release -j4 --target llama-server      # 10–20 min on a Pi 5
pi$ mkdir -p ~/models
```

Download the GGUFs from Liquid AI's Hugging Face org (search "LFM2.5-VL-450M GGUF" and
"LFM2.5-230M GGUF"). The **VL model needs two files**: the language GGUF and the
`mmproj-*.gguf` vision projector. Start with Q8_0 for both; keep a Q4_K_M of the 230M
handy in case the planner is slow.

Edit the paths at the top of `scripts/start_models.sh`, then:

```bash
pi$ bash scripts/start_models.sh &
pi$ sleep 20; curl -s localhost:8080/health; curl -s localhost:8081/health   # {"status":"ok"}
```

Benchmark the planner with a realistic request (this is exactly what the backend sends):

```bash
pi$ PEARL_STUB=1 python3 - <<'EOF'
import json, time, requests
from pearl.planner.planner_client import PlannerClient
c = PlannerClient()
ctx = {"target":"person in a red jacket","mission":"SEARCHING","vision":{"match":False,"conf":0.1,"bearing_deg":None},
       "last_sighting_age_s":None,"zones_searched":[],"zones_unsearched":["zone_a","zone_b","zone_c","zone_d"],
       "battery_pct":80,"recall":False,"safety_stop":False,"goals":{"active":[],"pending":0,"recent":[]}}
for i in range(5):
    d = c.decide(ctx); print(i, d.ok, d.latency_ms, "ms", [(x.name, x.arguments, x.priority) for x in d.calls], d.error, d.dropped)
EOF
```

First call is slow (prompt cache cold); calls 2–5 are what matters. Targets:
**planner ≤ 2000 ms**, and it should return `explore_until_found` for that context.
If it doesn't return sensible calls, try `PLANNER_PROMPT_VERSION="v1b"` before anything else.

Benchmark the VLM with a real photo:

```bash
pi$ PEARL_STUB=1 python3 - <<'EOF'
import time
from pearl.vision.vlm_client import VlmClient
c = VlmClient()
jpg = open("test.jpg","rb").read()          # any 320-px-wide JPEG, ideally of your actual target
for i in range(3):
    r = c.verify(jpg, "person in a red jacket", time.time()); print(r.match, r.confidence, r.bbox, r.error)
EOF
```

Target: **≤ 3000 ms** per call. If slower: `-t 3` for the VLM while ROS is off is fine for
the benchmark, but you'll be back to 2 threads on the car; set `VISION_VLM_INTERVAL_S`
to about 1.5× the measured latency.

Watch `top` while running: this is your first look at how much CPU headroom the models
leave.

**Done when:** both servers answer `/health`, the planner produces a valid tool call in
under ~2 s warm, the VLM answers in under ~3 s.

---

## Phase 5 — Map the arena and set zones/geofence (1 h)

Drive the robot around your test area with Hiwonder's own tools (SLAM running, their
app or teleop) and save the map the way their docs describe. Then, with the map loaded and
localisation running, park the robot at each spot you care about and read its pose:

```bash
pi$ ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -A1 Translation
```

Fill in `config.py`:
- `ZONES`: 3–6 named points spread over the area. Yaw = direction the camera should face on arrival.
- `GEOFENCE_POLYGON`: the corners of the area, **inside** the walls by ~0.3 m. Walk the
  robot to each corner and read the pose. The safety floor stops the chassis within
  `GEOFENCE_MARGIN_M` of this edge, so don't make it hug the wall.
- `CONCEALMENT_POINTS`: one or two spots behind furniture. Must be inside the fence.
- `HOME_POSE`: where it starts (unused by v1, but record it).

Sanity check: every zone and concealment point must be inside the polygon, or
validation will reject those goals with `outside_geofence`:

```bash
pi$ PEARL_STUB=1 python3 -c "
from pearl import config
from pearl.geometry import point_in_polygon as p
print({z: p(x,y,config.GEOFENCE_POLYGON) for z,(x,y,_) in config.ZONES.items()})
print([p(x,y,config.GEOFENCE_POLYGON) for x,y,_ in config.CONCEALMENT_POINTS])"
```

**Done when:** everything prints `True`.

---

## Phase 6 — Wire the safety gate and disable YOLO (30–60 min)

Two edits to the **Hiwonder** launch configuration, not to this project:

**6a. Disable the stock YOLO / perception node.** Find it in their launch file (grep for
`yolo`, `detect`, `perception`) and comment it out or set its launch argument to false.
Leave lidar obstacle avoidance and everything else running. The VLM replaces YOLO's job;
running both wastes the CPU you don't have.

**6b. Route Nav2's velocity through the safety gate.** Nav2's `controller_server` publishes
`cmd_vel`. You want it to publish to `/cmd_vel_nav` instead, and let `Ros2Robot` relay to
the real motor topic. In the Nav2 launch (or a wrapper launch), add a remapping on the
controller_server node:

```python
remappings=[('cmd_vel', '/cmd_vel_nav')]
```

If Hiwonder's Nav2 goes through a velocity smoother, remap the *smoother's* output instead
(it's the last node before the motor topic). Verify with `ros2 topic info -v
/controller/cmd_vel` (or whatever `cmd_vel_motor` is): after the change, the only
publisher to the motor topic while Nav2 is idle should be… nobody until the backend starts.

**If you can't get the remap working on day one:** set `SAFETY_GATE_CMD_VEL = False` in
`config.py`. The floor will publish zeros against Nav2 instead. It still stops the robot;
it's just uglier. Come back to this.

Then set the `TOPICS["cmd_vel_nav"]` value to match whatever you remapped to.

**Done when:** with Nav2 running and no goal active, `ros2 topic echo /cmd_vel_nav` is
silent, and `ros2 topic info -v <cmd_vel_motor>` shows no Nav2 publisher.

---

## Phase 7 — First real run: robot ON A STAND, wheels off the ground (1–2 h)

Order of startup, always:

1. Stock stack (SLAM/AMCL + Nav2 + camera, YOLO off).
2. `bash scripts/start_models.sh &` — wait for both `/health`.
3. `PEARL_STUB=0 python3 run.py`

Watch the terminal for the first 10 seconds. What you want to see:

```
backend.starting stub_mode=False
ros.started
health.change component=ros to=ok
health.change component=lidar to=ok
health.change component=camera to=ok
health.change component=battery to=ok
health.change component=nav2 to=ok
safety.stop active=True reason=lidar_timeout      <- normal for the first second
safety.stop active=False
backend.ready
```

Then from the laptop:

```bash
laptop$ curl -s http://<PI_IP>:8000/api/state | python3 -m json.tool
```

Every entry under `health` should be `ok` and `safety.stop_active` should be `false`.
Work down this list for anything that isn't:

| Symptom | Cause / fix |
|---|---|
| `lidar: down`, `safety reason=lidar_timeout` forever | Wrong topic, or QoS mismatch. `ros2 topic info -v /scan` → if publisher is BEST_EFFORT we match; if RELIABLE, change `sensor_qos` in `ros2_robot.py` to RELIABLE. |
| `camera: down` | Wrong topic or `CAMERA_USE_COMPRESSED` doesn't match what's published. `ros2 topic hz <topic>` to confirm it's actually publishing. |
| `battery: down` | Wrong topic/type. Echo it; fix `battery_msg_to_percent_volts`. |
| `nav2: down` | Nav2 not fully up (lifecycle nodes take ~10 s), or wrong action name. `ros2 action list`. |
| `pose: null` | TF lookup failing: `ros2 run tf2_ros tf2_echo map base_footprint`. If `map` frame doesn't exist, localisation isn't running. |
| `vision_model: down` / `planner_model: down` | `curl localhost:8080/health`. Servers not started or crashed — check `logs/llama_*.log`. |
| Python traceback on startup mentioning `get_message` | A string in `MSG_TYPES` is wrong. Phase 3 check script. |
| `ModuleNotFoundError: rclpy` | You're not in the ROS environment. Phase 1. |

Once health is green, still on the stand:

```bash
# 1. camera really is the robot's camera
laptop$ open http://<PI_IP>:8000/camera/stream

# 2. safety floor reacts: wave your hand 20 cm in front of the lidar
laptop$ watch -n0.5 'curl -s http://<PI_IP>:8000/api/state | python3 -c "import json,sys; print(json.load(sys.stdin)[\"safety\"])"'
#    → stop_active flips true with reason lidar_obstacle, back to false when you remove it

# 3. the VLM sees your target: stand in front of the camera in the red jacket
laptop$ curl -s -X POST -H 'Content-Type: application/json' -d '{"target":"person in a red jacket"}' http://<PI_IP>:8000/api/target
laptop$ watch -n1 'curl -s http://<PI_IP>:8000/api/vision'
#    → match true, confidence ≥0.6, a bbox, bearing_deg that changes sign as you move left/right

# 4. a nav goal goes through the action layer to Nav2 (wheels spin in the air)
laptop$ curl -s -X POST -H 'Content-Type: application/json' \
   -d '{"action":"navigate_to_zone","params":{"zone_id":"zone_a"},"priority":50}' http://<PI_IP>:8000/api/debug/goal
laptop$ curl -s http://<PI_IP>:8000/api/goals | python3 -m json.tool | head -40
#    → active[0].feedback.distance_remaining_m should be populated within a few seconds
#    → wheels should turn. Then:
laptop$ RECALL
#    → wheels stop, goal shows CANCELED / recalled, a priority-95 hold_position is active
```

While this runs, on the Pi: `top -H` and `vcgencmd measure_temp`. If the temperature
passes 80 °C you're going to throttle; get a fan on it.

Note which vision/planner latencies you actually see:
```bash
pi$ jq -c 'select(.event=="planner.decision") | .latency_ms' logs/pearl.jsonl | tail -20
pi$ jq -c 'select(.event=="vision.result") | .latency_ms' logs/pearl.jsonl | tail -20
```
Set `VISION_VLM_INTERVAL_S` and `PLANNER_INTERVAL_S` to ~1.5× those numbers.

**Done when:** health all green, safety stop reacts to your hand, VLM matches you, a
debug nav goal makes the wheels turn, RECALL stops them.

---

## Phase 8 — Wheels down, small fence, hand on RECALL (1 h)

Shrink the geofence for this phase: edit `GEOFENCE_POLYGON` to a 2 × 2 m box around the
start pose, and put `ZONES` corners inside it. Clear floor, nothing fragile.

1. Restart the backend (config is read at start).
2. `POST /api/debug/goal` a `hold_position` — nothing should move.
3. `POST /api/debug/goal` a `navigate_to_zone` to a zone 1 m away. Robot drives, stops,
   goal → `SUCCEEDED`, zone appears in `zones.searched`.
4. Send it toward the fence edge — pick a zone right at the boundary. Expect the safety
   floor to stop it with `reason=geofence` before it crosses, and the goal to end
   `ABORTED / safety_stop`.
5. Put a box in its path mid-route. Expect `lidar_obstacle` stop, and either Nav2 replans
   (goal continues) or the goal aborts with `safety_stop`. Both are acceptable.
6. Run two debug goals at priority 20 then 60: the second should preempt the first
   (`ABORTED / preempted_by_higher_priority`) and the robot should visibly change heading.

If the robot lurches or oscillates when the safety stop releases, raise
`LIDAR_RESUME_DISTANCE_M` (more hysteresis).

**Done when:** all six behave as described and you never had to touch the robot physically.

---

## Phase 9 — First autonomous mission (30 min)

Restore the real `GEOFENCE_POLYGON` and `ZONES`. Restart. Stand somewhere in the arena
in your target outfit.

```bash
laptop$ curl -s -X POST -H 'Content-Type: application/json' -d '{"target":"person in a red jacket"}' http://<PI_IP>:8000/api/target
laptop$ watch -n1 'curl -s http://<PI_IP>:8000/api/goals | python3 -c "import json,sys; d=json.load(sys.stdin); print([(g[\"action\"],g[\"feedback\"].get(\"percent_complete\")) for g in d[\"active\"]]); [print(h[\"action\"],h[\"state\"],h[\"reason\"]) for h in d[\"history\"][:6]]"'
```

Expected arc: `explore_until_found` EXECUTING → zones ticking off → `SUCCEEDED /
target_found` → `approach_target` → `SUCCEEDED` → `hold_position`. Afterwards, read the
trace:

```bash
pi$ jq -c 'select(.event=="planner.decision") | {calls, dropped, latency_ms}' logs/pearl.jsonl
pi$ jq -c 'select(.event=="goal.transition")   | {action, frm, to, reason}' logs/pearl.jsonl
pi$ jq -c 'select(.event=="goal.rejected")' logs/pearl.jsonl     # what the planner tried that was refused
```

`goal.rejected` lines are the most informative thing on the first run — they tell you
where the small model's idea of the world disagrees with the validator. That feeds
directly into prompt `v2`.

---

## Phase 10 — Make it survive a reboot (30 min)

Only after Phase 9 works. Two systemd units (NATIVE install; for DOCKER put the equivalent
in the container's entrypoint or a `docker exec` in the unit's `ExecStart`).

`/etc/systemd/system/pearl-models.service`:
```ini
[Unit]
Description=llama.cpp model servers for Pearl
After=network.target
[Service]
User=pi
WorkingDirectory=/home/pi/pearl_backend_v1
ExecStart=/bin/bash scripts/start_models.sh
Restart=on-failure
[Install]
WantedBy=multi-user.target
```

`/etc/systemd/system/pearl-backend.service`:
```ini
[Unit]
Description=Pearl Backend V1 (search-and-locate)
After=pearl-models.service <hiwonder-stack>.service
Requires=pearl-models.service
[Service]
User=pi
WorkingDirectory=/home/pi/pearl_backend_v1
Environment=PEARL_STUB=0
Environment=PEARL_DEBUG=0
ExecStartPre=/bin/sleep 20
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source <hiwonder_ws>/install/setup.bash && exec python3 run.py'
Restart=on-failure
RestartSec=5
[Install]
WantedBy=multi-user.target
```

```bash
pi$ sudo systemctl daemon-reload && sudo systemctl enable --now pearl-models pearl-backend
pi$ journalctl -u pearl-backend -f
```

Note `PEARL_DEBUG=0` — the debug endpoints let anyone on the WiFi drive the robot.
Turn them off once you're not using them.

---

## Dashboard connection

The dashboard runs on a laptop on the same WiFi and talks to `http://<PI_IP>:8000`.
CORS is fully open, `OPTIONS` preflights return 204, every payload has `timestamp`.
Give the Pi a static DHCP lease on your router so the IP stops changing.

---

## Tuning by symptom

| You see | Change |
|---|---|
| planner cycles slower than `PLANNER_INTERVAL_S` | Q4_K_M planner quant; `-c 1024`; `PLANNER_PROMPT_VERSION="v1b"`; raise interval to 3 s |
| planner timeouts (`planner.error timeout`) | raise `PLANNER_TIMEOUT_S`; check `--cache-reuse` is on; check `top` for what's stealing CPU |
| planner emits nonsense / many `dropped` | try `PLANNER_CONSTRAIN_MODE="gbnf"`; shorten prompt; check llama-server supports `response_format json_schema` (it has since early 2024) |
| VLM false positives | raise `VISION_MATCH_CONFIDENCE_MIN` to 0.7; make the target description more specific |
| VLM misses obvious target | lower to 0.5; raise `VISION_VLM_INPUT_WIDTH` to 448 (costs latency) |
| tracker drifts onto background | lower `VISION_TRACKER_MAX_MISSES`; raise `VISION_TRACKER_SCORE_MIN` |
| robot stops constantly for nothing | `LIDAR_STOP_DISTANCE_M` too high for your lidar's noise, or `LIDAR_MIN_VALID_M` too low (self-hits) — echo `/scan` and look at the minimum |
| robot stops at the fence but shouldn't | `GEOFENCE_MARGIN_M` too big, or fence polygon too tight |
| `approach_target` overshoots / stops short | `CAMERA_HFOV_DEG` or `LIDAR_YAW_OFFSET_DEG` wrong (bearing→range lookup misses) |
| `insufficient_battery` too early | fix `BATTERY_CELLS`; check the voltage in `/api/state` against a multimeter |
| MJPEG stutters and everything else slows | lower `MJPEG_FPS` to 4, `MJPEG_WIDTH` to 240; only one browser tab |
| Pi throttles (`vcgencmd get_throttled` ≠ 0x0) | active cooler; fewer llama threads |

---

## What to keep in your pocket during every run

```bash
alias RECALL='curl -s -X POST http://<PI_IP>:8000/api/recall'
alias STATE='curl -s http://<PI_IP>:8000/api/state | python3 -m json.tool'
alias GOALS='curl -s http://<PI_IP>:8000/api/goals | python3 -m json.tool | head -80'
alias HEALTH='curl -s http://<PI_IP>:8000/api/health'
```

And the robot's physical power switch. RECALL depends on WiFi and a Python process;
the switch doesn't.
