# MentorPi search-and-locate backend

Onboard software for an autonomous search-and-locate robot car (Hiwonder MentorPi, Raspberry Pi 5, ROS2 Humble, Python/rclpy).

A person types a target description ("person in a red shirt") into a remote dashboard. From then on the robot looks for the target, checks sightings with a local vision model, and turns or moves toward it on its own. The only human inputs are **set target** and **recall/stop**.

**Status:** runs on the real robot and in stub mode (no robot needed). The turn-only "watch me" demo works on the car. 98 automated tests pass.

```
                        ┌──────────────────────────────┐
  dashboard (laptop) ──►│ Layer 5  bridge/http_server  │ Flask, CORS, MJPEG
                        └──────────────┬───────────────┘
                                       │ reads SharedState + GoalRegistry
┌──────────────────┐    ┌──────────────▼───────────────┐    ┌───────────────────┐
│ Layer 1 vision   │───►│ Layer 2 planner/loop         │───►│ Layer 3 actions   │
│ VLM + tracker    │    │ rules | llm | trained model  │    │ manager/executors │
└────────┬─────────┘    └──────────────────────────────┘    └─────────┬─────────┘
         │                                                            │ motion
         │                                                            │ commands
         │              ┌─────────────────────────────────────────────▼───────┐
         └─────────────►│ robot/interface.py  ←  ros2_robot.py | stub_robot.py │
                        └──────────────────────────┬──────────────────────────┘
                        ┌──────────────────────────▼──────────────────────────┐
                        │ Layer 4 safety/safety_floor.py  (beneath everything) │
                        └─────────────────────────────────────────────────────┘
```

## The five layers

| Layer | What it does |
|---|---|
| 1. Vision | Every ~1.5 s, sends the newest camera frame to the vision model: "is the target in this picture?" Answer: match, confidence, box. A cheap tracker follows the box between checks. Any error means "no match". |
| 2. Planner | Every ~2 s, looks at a short summary of the robot's state and picks the next action. See [The planner](#the-planner) below. Any failure means "hold still". |
| 3. Actions | Runs each goal through a lifecycle (pending, executing, succeeded, aborted...). Handles priorities, time limits, resource locks and live feedback. |
| 4. Safety floor | Lidar obstacle stop and lidar-silence stop. Not a goal, cannot be cancelled or outranked. |
| 5. HTTP bridge | The Flask server the dashboard talks to (port 8000). |

> **Safety rule:** the planner decides what to try. It never decides whether it is safe. The safety floor stops the chassis directly, and the planner cannot turn it off.

## Folder layout

| Path | What it does |
|---|---|
| `run.py` | Entry point. `MENTORPI_STUB=1` (default) or `0` for the real robot. |
| `mentorpi/config.py` | Every topic, port, threshold and interval. Robot-specific values are marked `TODO(hw)`. Edit by hand on the robot; never overwrite it with a copy. |
| `mentorpi/app.py` | Builds the layers, starts the threads, handles shutdown. |
| `mentorpi/state.py` | `SharedState`: one lock, snapshots for the dashboard and the planner. |
| `mentorpi/logging_setup.py` | Text log plus JSON-lines log (`logs/mentorpi.jsonl`). |
| `mentorpi/geometry.py` | Polygon tests, yaw/quaternion helpers. |
| `mentorpi/robot/interface.py` | `RobotInterface`: the only way any layer touches hardware. |
| `mentorpi/robot/ros2_robot.py` | Real robot adapter (rclpy). Only loaded when `STUB_MODE=False`. |
| `mentorpi/robot/stub_robot.py` | Fake robot, fake vision model and fake planner. No ROS2 needed. |
| `mentorpi/vision/vlm_client.py` | Vision model call, strict JSON parsing, box to bearing angle. |
| `mentorpi/vision/tracker.py` | Simple tracker between vision checks. |
| `mentorpi/vision/pipeline.py` | Vision thread. |
| `mentorpi/planner/rule_planner.py` | The rule-based planner. |
| `mentorpi/planner/planner_client.py` | The model-based planner client (llama.cpp). Never raises. |
| `mentorpi/planner/prompts.py` | Versioned system prompts for the model-based planner. |
| `mentorpi/planner/tools.py` | Tool list and JSON schema for constrained output. |
| `mentorpi/planner/grammar.gbnf` | Alternative output grammar. |
| `mentorpi/planner/loop.py` | Planner thread: build context, decide, submit, fall back on failure. |
| `mentorpi/actions/goal.py` | `Goal`, goal states, reason strings, the action list. |
| `mentorpi/actions/validation.py` | Checks a goal at submission and rejects it with a reason. |
| `mentorpi/actions/registry.py` | Goal history. The only place `goal.state` changes. |
| `mentorpi/actions/executors.py` | One state machine per action (map-based driving with Nav2). |
| `mentorpi/actions/reactive.py` | No-map driving and turn-only mode. |
| `mentorpi/actions/manager.py` | Layer 3 core: locks, preemption, time limits, queue, recall. |
| `mentorpi/safety/safety_floor.py` | Layer 4. |
| `mentorpi/bridge/http_server.py` | Layer 5 server. |
| `mentorpi/bridge/dashboard_adapter.py` | Converts internal data to the exact format the dashboard expects. |
| `scripts/discover_hw.sh` | Prints the ROS topic names and other values `config.py` needs. |
| `scripts/start_models.sh` | Starts the llama.cpp model servers. |
| `tests/` | 98 pytest tests. No ROS2 or robot needed. |

## Running

### Stub mode (laptop, no robot)

```bash
pip install -r requirements.txt
python3 run.py
# another shell
curl -s localhost:8000/api/state | python3 -m json.tool
curl -s -X POST -H 'Content-Type: application/json' -d '{"target":"person in a red jacket"}' localhost:8000/api/target
curl -s -X POST localhost:8000/api/recall
python3 -m pytest -q
```

The fake world makes the fake target appear, the robot goes to it, and random "obstacles" trigger safety stops so the failure paths get exercised.

### Real robot

The robot runs ROS2 inside a Docker container called `MentorPi`. The code folder is shared:

| From | Path |
|---|---|
| Pi terminal | `~/mentorpi_backend` |
| Inside Docker | `/home/ubuntu/shared/mentorpi_backend` |

```bash
# 1. On the Pi host: start the model server(s)
bash scripts/start_models.sh &

# 2. Inside Docker: check the lidar and motors are running (see "Before you start")
docker exec -it MentorPi bash
cd /home/ubuntu/shared/mentorpi_backend

# 3. Start the backend
MENTORPI_STUB=0 nohup python3 run.py > /tmp/backend.log 2>&1 &
```

**Turn-only demo:** add `MENTORPI_TURN_ONLY=1` to the front of the command in step 3.

### Environment settings

| Variable | Default | Meaning |
|---|---|---|
| `MENTORPI_STUB` | `1` | 1 = fake robot, 0 = real robot |
| `MENTORPI_TURN_ONLY` | `0` | 1 = the car never drives forward, it only turns in place |
| `MENTORPI_PLANNER` | from config | Overrides `PLANNER_BACKEND` for one run |
| `MENTORPI_NAV_MODE` | `reactive` on the robot, `nav2` in stub mode | How the car moves (see [Driving modes](#driving-modes)) |
| `MENTORPI_HTTP_PORT` | `8000` | Dashboard port |
| `MENTORPI_VLM_URL` | `http://127.0.0.1:8082/v1/chat/completions` | Vision model server |
| `MENTORPI_PLANNER_URL` | `http://127.0.0.1:8081/v1/chat/completions` | Planner model server |
| `MENTORPI_LOG_LEVEL` | `INFO` | Log detail |
| `MENTORPI_DEBUG` | `1` | 0 turns off the debug endpoints (do this on a shared network) |

### Before you start (real robot)

- After a reboot, Hiwonder's own software already runs the motor chain. Do not start a second copy. Check that `/controller/cmd_vel` shows `Subscription count: 1`.
- Never run two copies of the lidar driver. If there is no scan topic, Hiwonder's lidar program may be stuck: `pkill -f ldlidar_stl_ros2_node`, then start one lidar launch.
- Keep the area within 30 cm in front of the lidar clear, including cables. Anything there triggers a safety stop, which also blocks turning.
- The Pi's home WiFi address can change. Check `hostname -I` and update the dashboard.
- Run in the Docker container with `nohup` so a dropped SSH connection does not kill the backend.

## Driving modes

Set with `NAV_MODE` in `config.py` (or `MENTORPI_NAV_MODE`).

| Mode | When to use it | How it works |
|---|---|---|
| `reactive` | Unfamiliar room, no map (the default on the robot) | Short, slow, timed moves. Stops to look between moves. Zones are not used. |
| `nav2` | A mapped room with SLAM and Nav2 | Goals go to Nav2's `NavigateToPose`. Uses zones and a geofence. |

### Turn-only mode

`MENTORPI_TURN_ONLY=1` (reactive mode, mecanum chassis only). The car never drives forward. It spins in place to keep facing the person while the person walks around it.

- **Approach becomes "watch".** It turns toward the target and never "arrives".
- It trusts only the vision model's answers, not the tracker (the tracker was wrong right after the car turned).
- **Aim ahead.** It estimates how fast the person is circling and turns to where they will be at the next look (only above 1.5 deg/s, at most 30 deg ahead).
- **If it loses the person:** it looks once more, then turns 45 deg at a time in the direction they were moving, up to 8 turns (a full circle), then gives up (`target_lost`).
- Each look waits up to 25 s for an answer from the vision model.
- Explore only turns. Hide only turns away. Any forward move is refused.
- An ackermann chassis is refused with a clear message.
- Settings are constants at the top of `mentorpi/actions/reactive.py`.

**What to expect:** the vision model takes about 5 s per check on the robot. The person should walk a few small steps, then pause. About 10 s of pause is needed at 5 s per check, and about 3 s at 3 s per check. If the car loses the person, they should stand still and it will find them.

## The planner

The planner picks the next action every ~2 s. It reads a short state summary (target, what the vision model sees, last sighting age, battery, behavior mode, active goals, last 5 goal results) and returns up to two tool calls with priorities.

All planners share one interface (`decide(context)` and `healthy()`), so switching is one setting. Every planner's output goes through the same validator and the same safety floor.

### Planner backends

Set `PLANNER_BACKEND` in `config.py`, or `MENTORPI_PLANNER=<name>` for one run.

| Backend | What it is | Status |
|---|---|---|
| `rules` | Fixed decision rules in code (below). Answers in about 0 ms. No model server needed. | Default for the demo. |
| `llm` | A model served by llama.cpp on port 8081, with grammar-constrained output. | Used for the trained planner (below). |

### The rules (`rules` backend)

First match wins:

1. Battery below 20%: hide (priority 70).
2. Target seen, behavior is HIDE: hide (priority 60), or hold if already hidden.
3. Target seen, behavior is FOLLOW: approach to 1.5 m (priority 60), or hold if it just arrived.
4. Target seen less than 20 s ago: scan toward its last bearing (priority 40).
5. A goal is already running: do nothing new.
6. Zones left to search: explore for 180 s (priority 20).
7. Otherwise: hold for 10 s (priority 10).

### The trained planner (`llm` backend)

> **Fill in before use.** These values come from the training run and are not recorded elsewhere in the project.

| Item | Value |
|---|---|
| Base model | `<FILL IN>` |
| Trained model file (in `~/models`) | `<FILL IN>.gguf` |
| Quantization | `<FILL IN>` |
| Training data | `<FILL IN: e.g. states labeled by the rule planner>` |
| Accuracy vs the rules (all states) | `<FILL IN>` |
| Lowest per-rule accuracy | `<FILL IN>` |
| Latency on the Pi (warm) | `<FILL IN>` ms |
| Prompt version | `<FILL IN>` (`PLANNER_PROMPT_VERSION`) |
| Server flags | port 8081, `-t <FILL IN>`, `-c 2048`, `--parallel 1`, `--jinja` |

To run it:

```bash
# Pi host: start the planner server with the trained model, then in Docker:
MENTORPI_PLANNER=llm MENTORPI_STUB=0 python3 run.py
```

How the model-based planner works:

- It calls the llama.cpp server at `/v1/chat/completions`. The system prompt comes from `prompts.py` (version set by `PLANNER_PROMPT_VERSION`). The user message is the state JSON.
- **Constrained output.** `PLANNER_CONSTRAIN_MODE = "json_schema"` forces the answer into `{"tool_calls":[{"name","arguments","priority"}]}` with legal tool names and arguments. Other modes: `"gbnf"` and `"native"`.
- Temperature 0, `PLANNER_MAX_TOKENS` 160, at most 2 calls per cycle, `PLANNER_TIMEOUT_S` 15.
- Every call is checked a second time against the action rules in `goal.py`.
- On a timeout, a dead server or an unreadable answer, `decide()` returns `ok=False` and the loop submits `hold_position` (3 s, priority 10).
- Each decision is logged with the prompt version, calls, dropped calls and latency.

**Planner lessons from testing the untrained 230M model:** it always produced valid JSON but picked the wrong action (it got the first decision of a mission right 0 of 6 times, and took up to 9 s per call). That is why the rule planner is the default. Use the rule planner as the answer key when you score any model: it can label as many test states as you need.

### What the planner can ask for

| Action | Parameters | Uses | Moves the car? |
|---|---|---|---|
| `navigate_to_zone` | `zone_id` | chassis | yes (`nav2` mode only) |
| `explore_until_found` | `ttl_seconds` (10-900) | chassis | yes (turns only in turn-only mode) |
| `approach_target` | `standoff_meters` (0.5-5.0), needs a recent sighting | chassis | yes (watch only in turn-only mode) |
| `reposition_for_concealment` | none | chassis | yes |
| `hold_position` | `seconds` (1-120, default 5) | chassis | no |
| `scan_with_camera` | `pan_deg`, optional `tilt_deg` | pan-tilt | no (the pan-tilt servo is not wired up yet, so this does nothing) |

Planner priorities are 0-79. Recall uses 95. The safety floor has no priority because it is not a goal.

## The action layer

A goal is `(action, params, priority, ttl, evidence_ts)`.

1. `submit()` checks it and rejects it with a reason if it is not allowed.
2. If the resources it needs (`chassis`, `pan_tilt`) are free, or held by a lower-priority goal, it starts immediately and cancels the lower one (`preempted_by_higher_priority`). Otherwise it waits in a short queue.
3. A 5 Hz tick runs each goal, ends goals whose time limit has passed, and starts queued goals highest priority first.

**Rejected at submission:** `unknown_action`, `invalid_params`, `unknown_zone`, `outside_geofence`, `insufficient_battery`, `recall_active`, `priority_out_of_range`, `queue_full`, `duplicate_goal`, `no_target_set`, `no_recent_sighting`, `robot_pose_unknown`.

**Ended after starting:** `succeeded`, `ttl_expired`, `preempted_by_higher_priority`, `recalled`, `canceled_by_operator`, `new_target`, `path_blocked`, `unreachable`, `nav2_rejected_goal`, `nav2_server_down`, `safety_stop`, `all_zones_searched`, `target_lost`, `target_found`, `executor_exception`, `shutdown`.

## Safety floor

- **Lidar stop:** an obstacle closer than the stop distance in the front cone stops the chassis. This also blocks turning.
- **Lidar silence:** no lidar data for 1 s means stop.
- **Geofence** (`nav2` mode): leaving the fence area stops the chassis.
- **Motion gate:** the floor closes a gate on the motor command topic and sends zeros directly. In reactive mode, the backend's own motion commands also pass through the gate.
- **Fail closed everywhere:** a vision error means "not seen", a planner error means "hold", missing data means "stop".
- Hiwonder's own obstacle avoidance keeps running as an independent backup.
- **Recall** cancels everything, blocks the planner, and holds at priority 95. It does not drive home. The robot's power switch is the final stop.

## The vision model and model servers

Both models run in llama.cpp servers on the Pi host (not in Docker). Docker uses host networking, so the backend reaches them at `127.0.0.1`.

| Server | Model | Port | Notes |
|---|---|---|---|
| Vision | Liquid AI LFM2.5-VL-450M (Q8_0) plus its mmproj file | 8082 | `-t 2`, `-c 2048`, `--parallel 1`. Needs `--mmproj`. |
| Planner | See [The planner](#the-planner) | 8081 | `-t 1`, `-c 2048`, `--parallel 1`, `--jinja`. Only needed for the `llm` backend. |

- **Never use port 8080.** Hiwonder's camera web server already uses it.
- Health check: `curl -s localhost:8082/health` should return `{"status":"ok"}`.
- Logs: `logs/llama_vlm.log` and `logs/llama_planner.log`.
- Vision speed on the robot is about 5 s per check (target is about 3 s). To speed it up: use the rule planner and do not start the planner server, give the vision server `-t 3`, and make sure the power supply is good (a yellow lightning bolt on the Pi screen means low power). Check the speed with:

  ```bash
  grep vision.result /tmp/backend.log | grep -o "latency_ms=[0-9]*" | tail -10
  ```

- Set `VISION_VLM_INTERVAL_S` to about 1.5 times the measured speed.
- The vision accuracy test on the robot has not been run yet. The only accuracy test is the off-robot Colab comparison.

## HTTP contract

Port 8000. Every response has `Access-Control-Allow-Origin: *`. `OPTIONS /api/*` returns 204. Timestamps are integer milliseconds. The `dashboard_adapter.py` file is the only place that converts between the backend's own format and the dashboard's format.

| Endpoint | What it returns or does |
|---|---|
| `GET /api/state` | Mission, battery, pose, health of each component (`ros2_core`, `planner_llm`, `vision_llm`, `nav2`, `camera`), safety status, last planner decision |
| `GET /api/goals` | One active goal (or null) with live feedback, the queue, and the last 50 goals with status and reason |
| `GET /api/vision` | Latest vision result: match, confidence, box (`x_min,y_min,x_max,y_max`), timestamp |
| `GET /api/health` | Raw health and safety |
| `GET /camera/stream` | MJPEG, capped at `MJPEG_FPS` and `MJPEG_WIDTH` |
| `POST /api/target` | `{"target":"..."}` cancels goals and starts searching |
| `POST /api/behavior` | `{"mode":"follow"}` or `{"mode":"conceal"}` (use `follow` for the turn-only demo) |
| `POST /api/recall` | Cancels everything and holds |
| `POST /api/debug/goal`, `POST /api/debug/cancel_all`, `GET /api/debug/config`, `GET /api/debug/state_raw` | Bring-up only. Turn off with `MENTORPI_DEBUG=0`. |

To point the dashboard at the robot, set `ROBOT_BASE_URL` in the dashboard HTML to `http://<PI_IP>:8000` and click LIVE. There is no login on the bridge: use it on a trusted network only.

## Threads

| Thread | Cadence |
|---|---|
| Robot adapter (`ros-executor` or `stub-robot`) | callbacks / 10 Hz |
| `safety-floor` | 10 Hz and on every lidar message |
| `action-tick` | 5 Hz |
| `vision` | tracker 4 Hz, vision model every ~1.5 s or slower |
| `planner` | every 2 s |
| `http` | on demand |

Only the robot adapter thread calls rclpy. All loops sleep on `Event.wait()`, so there are no busy loops.

## Logs

`logs/mentorpi.jsonl`, one JSON object per line. Events: `planner.decision`, `planner.error`, `planner.fallback`, `goal.submitted`, `goal.rejected`, `goal.queued`, `goal.preempted`, `goal.transition`, `safety.stop`, `vision.result`, `vision.error`, `health.change`, `reactive.turn_only`, `mission.*`, `operator.*`.

```bash
jq -c 'select(.event=="goal.transition")' logs/mentorpi.jsonl
jq -c 'select(.event=="planner.decision") | {prompt_version, calls, latency_ms}' logs/mentorpi.jsonl
```

## Robot settings in `config.py`

Values set on this robot (check them with `scripts/discover_hw.sh` if anything changes):

| Setting | Value on this robot |
|---|---|
| `CHASSIS_VARIANT` | `mecanum` |
| `TOPICS["lidar_scan"]` | `/scan_raw` |
| Camera | raw image topic (compressed is off) |
| VLM URL | port 8082 |

Still to confirm or set if you use them: `TOPICS["pan_tilt_servo"]` and `Ros2Robot.set_pan_tilt()` (currently log only), and for `nav2` mode the `ZONES`, `GEOFENCE_POLYGON`, `CONCEALMENT_POINTS` and `HOME_POSE`. `grep -rn "TODO" mentorpi/` gives the full list.

## Tests

```bash
python3 -m pytest -q                                                 # all 98 tests
python3 -m pytest -q tests/test_reactive.py tests/test_turn_only.py  # driving tests: 34
```

## Known gaps

- Turn-only is simulated for speed. Real turns are timed, not measured, so they drift a little. Each new look corrects it.
- Vision model is slow (about 5 s per check), which sets how fast a person can walk.
- Driving forward on the real robot (non turn-only) has not been tried on the floor yet.
- Recall holds still; it does not drive home.
- Camera pan-tilt does nothing yet, so `scan_with_camera` has no effect.
- No map in reactive mode: no zones, no geofence, no real hiding spots. Hide just turns away.
- Tracker drifts on size changes and is only meant to bridge gaps between vision checks.
- Planner memory is the last 5 goal results only.
- The planner prompts do not cover every rule in the rule planner (for example "already hidden" and "goal already running"). Score a model on the covered states and on all states, and decide how before you run the test.
- Planner still runs during a safety stop. It is harmless because the floor blocks motion.
- Goal history is in memory. A restart loses it; the log file is the lasting record.
- No login on the HTTP bridge (trusted network only). Debug endpoints are on by default.
- No tests for the ROS adapter, the vision threading, or the Flask app itself.
- Battery percent is a straight-line voltage estimate.
