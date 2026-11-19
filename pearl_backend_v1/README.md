# Pearl Backend V1 — search-and-locate robot backend

Onboard software for an autonomous search-and-locate robot car (Hiwonder MentorPi,
Raspberry Pi 5, ROS2 Humble, Python/rclpy). A person types a target description into a
remote dashboard; from then on the robot explores, verifies sightings with a local VLM,
approaches, and repositions on its own. The only human inputs are *set target* and
*recall/stop*.

This is a **working skeleton**: it runs end-to-end in stub mode, has a tested action
layer, and marks every hardware-specific unknown with `TODO(hw)`. It has **not** been
run on the robot. See "Known gaps" at the bottom.

> **Placeholders — fill these in on your own copy, don't commit your real values.**
> This repo contains no real IP addresses, passwords or keys. Wherever you see one of
> these, replace it with your own value:
>
> | Placeholder | What goes there | How to find it |
> |---|---|---|
> | `<PI_IP>` | The robot's LAN IP address (e.g. `192.168.x.x`) | On the Pi: `hostname -I` |
> | `<hiwonder_ws>` | Path to the robot vendor's ROS2 workspace (the folder containing `install/setup.bash`) | See Phase 1 of `DEPLOY.md` |
> | `<hiwonder-stack>` | Name of the vendor's systemd service that starts the stock ROS2 stack | `systemctl list-units --type=service \| grep -iE 'ros\|hiwonder'` |
> | `TODO(hw)` in `pearl/config.py` | Topic names, zone coordinates, geofence, battery cells, etc. for *your* robot and *your* map | `scripts/discover_hw.sh` on the Pi |
>
> The SSH login for the Pi is the vendor default (see their docs) — change it, and never put it in this repo.
> The dashboard that talks to this backend is `pearl_ui.html`; put the same `<PI_IP>` in its `ROBOT_BASE_URL`.

```
                         ┌────────────────────────────┐
   dashboard (laptop) ──►│ Layer 5  bridge/http_server │ Flask, CORS, MJPEG
                         └─────────────┬──────────────┘
                                       │ reads SharedState + GoalRegistry
 ┌──────────────────┐   ┌──────────────▼──────────────┐   ┌──────────────────┐
 │ Layer 1 vision   │──►│ Layer 2 planner/loop         │──►│ Layer 3 actions   │
 │ VLM + tracker    │   │ LFM2.5-230M via llama.cpp    │   │ manager/executors │
 └────────┬─────────┘   └─────────────────────────────┘   └────────┬─────────┘
          │                                                         │ Nav2 NavigateToPose
          │             ┌───────────────────────────────────────────▼─────────┐
          └────────────►│ robot/interface.py  ←  ros2_robot.py | stub_robot.py │
                        └───────────────────────────┬───────────────────────────┘
                        ┌───────────────────────────▼───────────────────────────┐
                        │ Layer 4 safety/safety_floor.py  (beneath everything)   │
                        └────────────────────────────────────────────────────────┘
```

## File / module layout

| Path | What it does |
|---|---|
| `run.py` | Entry point. `PEARL_STUB=1` (default) or `0`. |
| `pearl/config.py` | **Every** topic, action, message type, port, threshold, interval, zone coordinate. All `TODO(hw)` placeholders live here. |
| `pearl/app.py` | Wires the layers, starts threads in order, handles SIGINT/SIGTERM. |
| `pearl/state.py` | `SharedState` — one lock, plain-dict snapshots for the bridge and compact context for the planner. |
| `pearl/logging_setup.py` | Text log to stderr + JSON-lines to `logs/pearl.jsonl` (`log_event(...)`). |
| `pearl/geometry.py` | Polygon tests, yaw↔quaternion. |
| `pearl/robot/interface.py` | `RobotInterface` — the only surface the layers use to touch hardware. |
| `pearl/robot/ros2_robot.py` | Real robot: rclpy node, executor thread, subscriptions, Nav2 action client, cmd_vel gate. Imported only when `STUB_MODE=False`. |
| `pearl/robot/stub_robot.py` | Synthetic robot + `StubVlm` + `StubPlanner`. No ROS2 anywhere. |
| `pearl/vision/vlm_client.py` | Layer 1. llama.cpp VLM call, fail-closed JSON parsing, bbox→bearing. |
| `pearl/vision/tracker.py` | Cheap classical tracker (template match on 160-px grayscale; KCF if contrib present). |
| `pearl/vision/pipeline.py` | Vision thread: tracker at 4 Hz, VLM re-verification every 1.5 s. |
| `pearl/planner/tools.py` | Tool vocabulary derived from `actions/goal.py ACTIONS`; JSON schema for constrained decoding. |
| `pearl/planner/grammar.gbnf` | Hand-written GBNF alternative to the JSON schema. |
| `pearl/planner/prompts.py` | **Versioned** system prompts (`PROMPT_VERSIONS`), selected by `config.PLANNER_PROMPT_VERSION`. |
| `pearl/planner/planner_client.py` | Layer 2. llama.cpp call, constrained decoding, tolerant parsing + schema validation. Never raises. |
| `pearl/planner/loop.py` | Planner thread: context → decide → submit; safe fallback on failure. |
| `pearl/actions/goal.py` | `Goal` dataclass, states, **reason strings**, `ACTIONS` vocabulary (params schema + resources). |
| `pearl/actions/validation.py` | Rejection-at-submission checks. |
| `pearl/actions/registry.py` | In-memory goal history (bounded, ≥50), the only place `goal.state` changes. |
| `pearl/actions/executors.py` | One non-blocking state machine per action; Nav2 wrapper (`_NavLeg`). |
| `pearl/actions/manager.py` | Layer 3 core: mutexes, priority preemption, TTL, queueing, recall, feedback. |
| `pearl/safety/safety_floor.py` | Layer 4. Lidar stop + geofence + lidar-timeout fail-closed. Not a tool, not a goal. |
| `pearl/bridge/http_server.py` | Layer 5. Fixed endpoint contract, CORS, MJPEG, own thread. |
| `scripts/discover_hw.sh` | Run on the Pi to print every identifier needed for `config.py`. |
| `scripts/start_models.sh` | Starts the two llama.cpp servers. |
| `DEPLOY.md` | Phase-by-phase bring-up guide for putting this on the robot. |
| `tests/` | pytest: action layer (25 tests), parsers, geometry. Runs without ROS2. |

## Running

### Stub mode (laptop, no robot, no ROS2, no model servers)

```bash
pip install -r requirements.txt
python3 run.py                     # PEARL_STUB defaults to 1
# in another shell
curl -s localhost:8000/api/state | python3 -m json.tool
curl -s -X POST -H 'Content-Type: application/json' -d '{"target":"person in a red jacket"}' localhost:8000/api/target
watch -n1 'curl -s localhost:8000/api/goals | python3 -m json.tool | head -60'
curl -s -X POST localhost:8000/api/recall
open http://localhost:8000/camera/stream          # MJPEG with the synthetic scene
python3 -m pytest -q                              # 51 tests, ~1.5 s
```

The synthetic world has a target near `zone_b`. After setting a target you will see:
`explore_until_found` → sighting → `approach_target` → `SUCCEEDED`, with random lidar
"obstacles" occasionally producing `ABORTED / safety_stop` so the failure paths are
exercised. `StubVlm`/`StubPlanner` also fail ~5%/2% of the time on purpose.

To develop prompts against the **real** llama.cpp servers while still using the synthetic
robot: `PEARL_STUB=1 PEARL_STUB_REAL_MODELS=1 python3 run.py`.

### Real robot (Pi 5)

```bash
source /opt/ros/humble/setup.bash          # plus the Hiwonder workspace setup.bash
# 1. stock stack running (SLAM, Nav2, camera). Then:
bash scripts/discover_hw.sh | tee hw_discovery.txt      # fill in config.py TODO(hw)
bash scripts/start_models.sh &                          # llama.cpp x2
PEARL_STUB=0 python3 run.py
```

Config via env: `PEARL_HTTP_PORT`, `PEARL_VLM_URL`, `PEARL_PLANNER_URL`,
`PEARL_LOG_LEVEL`. Everything else is edited in `config.py`.

## HTTP contract (fixed)

| Endpoint | Payload |
|---|---|
| `GET /api/state` | `timestamp, mission{state,target,recall_active}, battery{percent,voltage}, pose, zones{all,searched}, last_sighting, health{ros,camera,lidar,battery,nav2,vision_model,planner_model,http}, safety{stop_active,reason}, planner{last decision}` |
| `GET /api/goals` | `timestamp, active[] (with live feedback: distance_remaining_m, percent_complete, elapsed_s…), pending[], history[] (last 50, with state+reason+detail)` |
| `GET /api/vision` | `timestamp, match, confidence, bbox, bearing_deg, vlm_timestamp, ok, error, source(vlm|tracker), tracker{active,bbox}` |
| `GET /camera/stream` | `multipart/x-mixed-replace`, capped at `MJPEG_FPS`, `MJPEG_WIDTH` |
| `POST /api/target` | body `{"target": "..."}` → cancels live goals, resets zones, mission → SEARCHING |
| `POST /api/behavior` | body `{"mode": "follow"\|"conceal"}` → operator's standing instruction on contact; reflected in `/api/state` mission.behavior_mode |
| `POST /api/recall` | cancels everything, blocks planner, resets behavior, issues priority-95 `hold_position` |
| `POST /api/debug/goal`, `POST /api/debug/cancel_all`, `GET /api/debug/config`, `GET /api/debug/state_raw` | bring-up only (`PEARL_DEBUG=0` disables); submit a goal by hand, dump config, see the unmapped internal state |

These GET endpoints emit **exactly** the contract the `pearl_ui.html`
dashboard expects, produced by `bridge/dashboard_adapter.py`: **timestamps in integer
milliseconds**, components keyed `ros2_core / planner_llm / vision_llm / nav2 / camera`
each `{status, since, last_seen}`, a **single** `active` goal object (or null), a `queue`
array, a distinct `PREEMPTED` status, and `bbox` as `{x_min,y_min,x_max,y_max}`. The
backend's internal representation stays in seconds/arrays; the adapter is the only place
the two meet, so the UI never forces a change on the action layer. To point the dashboard
at the robot: edit `ROBOT_BASE_URL` in the HTML to `http://<PI_IP>:8000` and click LIVE.
`Access-Control-Allow-Origin: *` on every response; `OPTIONS /api/*` returns 204.

## The action layer in one paragraph

A goal is `(action, params, priority, ttl, evidence_ts)`. `submit()` validates it
(specific reason strings: `unknown_action, invalid_params, unknown_zone,
outside_geofence, insufficient_battery, recall_active, priority_out_of_range,
queue_full, duplicate_goal, no_target_set, no_recent_sighting, robot_pose_unknown`),
then arbitrates the resources it needs (`chassis`, `pan_tilt` — independent mutexes).
If every resource is free or held by a strictly lower priority goal it starts
**immediately, inside `submit()`**, aborting the previous holders with
`preempted_by_higher_priority`. Otherwise it queues (bounded). A tick thread at 5 Hz steps
executors, expires goals whose `evidence_ts + ttl` has passed (`ttl_expired`), and
promotes queued goals highest-priority-first. Terminal reasons after acceptance:
`succeeded, ttl_expired, preempted_by_higher_priority, recalled, canceled_by_operator,
path_blocked, unreachable, nav2_rejected_goal, nav2_server_down, safety_stop,
all_zones_searched, target_lost, target_found, executor_exception, shutdown`.
Planner priorities are 0–79; recall is 95; the safety floor has no priority because it
is not a goal.

## Safety invariant

> The model has wide discretion over strategy and parameters, and zero discretion over
> the safety layer.

Enforced structurally: the planner's tools are derived from `ACTIONS`; the safety floor
is not in `ACTIONS`, is not a goal, has no priority/TTL, no public disable, and stops the
chassis by closing the cmd_vel gate + publishing zeros directly — goal cancellation is
bookkeeping that happens afterwards. Lidar silence for 1 s ⇒ stop (fail closed). Stock
Hiwonder obstacle avoidance stays running as an independent override.

**Recommended wiring** (`SAFETY_GATE_CMD_VEL=True`): remap Nav2's controller output to
`TOPICS.cmd_vel_nav`; `Ros2Robot` relays it to `TOPICS.cmd_vel_motor` only while the
gate is open. If you can't remap, set it `False` and the floor publishes zeros against
Nav2 (works, ugly).

## Threading

| Thread | Owner | Cadence |
|---|---|---|
| `ros-executor` (×2 workers) / `stub-robot` | robot adapter | callbacks / 10 Hz |
| `safety-floor` watchdog | SafetyFloor | 10 Hz (also runs on lidar callback) |
| `action-tick` | ActionManager | 5 Hz |
| `vision` | VisionPipeline | 4 Hz tracker, 1.5 s VLM |
| `planner` | PlannerLoop | 2 s |
| `http` (+1/request) | werkzeug | on demand |

All shared state goes through `SharedState` / `GoalRegistry` / `ActionManager` locks.
No thread but `ros-executor` calls rclpy. All loops use `Event.wait()` — no busy loops.
Image data is held by reference; the only copies are the JPEG encodes (VLM input, MJPEG).

## Structured logs

`logs/pearl.jsonl`, one JSON object per line. Events: `planner.decision` (context,
calls, dropped, raw, latency, prompt_version), `planner.error`, `planner.fallback`,
`goal.submitted`, `goal.rejected`, `goal.queued`, `goal.preempted`, `goal.transition`
(from/to/reason/detail), `safety.stop`, `vision.result`, `vision.error`,
`health.change`, `mission.*`, `operator.*`.

```bash
jq -c 'select(.event=="goal.transition")' logs/pearl.jsonl
jq -c 'select(.event=="planner.decision") | {prompt_version, calls, latency_ms}' logs/pearl.jsonl
```

## Starting the llama.cpp servers

`scripts/start_models.sh` — edit the GGUF paths. VLM on :8080 (needs `--mmproj`),
planner on :8081. Both expose `/v1/chat/completions` and `/health`. The backend sends
`response_format: {type: json_schema}` so llama.cpp grammar-constrains the output
server-side (`PLANNER_CONSTRAIN_MODE="json_schema"`); alternatives are `"gbnf"`
(`planner/grammar.gbnf`) and `"native"` (OpenAI `tools`, requires `--jinja` and a chat
template with tool support). Thread counts in the script are a guess — the Pi has 4 cores
shared with SLAM/Nav2.

## Full list of TODO(hw) — things only the robot can tell you

Everything below is in `pearl/config.py` unless noted. `grep -rn "TODO" pearl/` gives
the authoritative list.

1. `CHASSIS_VARIANT` — ackermann or mecanum.
2. `TOPICS["cmd_vel_motor"]` — the topic the motor controller actually listens on.
3. `TOPICS["cmd_vel_nav"]` — where you remap Nav2's controller output (see Safety).
4. `TOPICS["lidar_scan"]`, `LIDAR_YAW_OFFSET_DEG`, `LIDAR_MIN_VALID_M` — check `angle_min` and whether angle 0 is forward.
5. `TOPICS["camera_image"]` / `["camera_image_compressed"]`, `CAMERA_USE_COMPRESSED`.
6. `TOPICS["battery"]`, `MSG_TYPES["battery"]`, `battery_msg_to_percent_volts()`, `BATTERY_CELLS`.
7. `TOPICS["odom"]`, `POSE_SOURCE`, `FRAME_MAP`, `FRAME_BASE`.
8. `TOPICS["pan_tilt_servo"]`, `MSG_TYPES["pan_tilt_servo"]`, and `Ros2Robot.set_pan_tilt()` in `robot/ros2_robot.py` (currently logs only).
9. `NAV2_ACTION_NAME`, `NAV2_ACTION_TYPE` — probably right, verify.
10. `ZONES`, `GEOFENCE_POLYGON`, `CONCEALMENT_POINTS`, `HOME_POSE` — coordinates from your SLAM map.
11. `CAMERA_HFOV_DEG` — for bbox→bearing.
12. `Ros2Robot._decode()` — add the camera's raw encoding if it is not bgr8/rgb8/mono8 (only matters if `CAMERA_USE_COMPRESSED=False`).
13. `planner/grammar.gbnf` — zone ids hard-coded; regenerate if `ZONES` changes (only for `PLANNER_CONSTRAIN_MODE="gbnf"`).
14. `scripts/start_models.sh` — GGUF paths and thread counts.

## Known gaps / deliberately unimplemented in v1

- **Never run on hardware.** `ros2_robot.py` compiles and follows rclpy Humble APIs but is untested; expect small fixes (QoS mismatches on `/scan` are the usual first one).
- **Recall = hold.** `HOME_POSE` exists but recall does not drive home yet (`ActionManager.recall()` has the TODO).
- **Concealment is naïve**: nearest configured point, no line-of-sight reasoning against the map.
- **Approach range** comes from the lidar bin at the bearing (or a 3 m fallback), not the depth camera. People are poor lidar targets; the depth camera would be better.
- **Nav2 failure reasons** (`path_blocked` vs `unreachable`) are a heuristic from progress. Humble's `NavigateToPose` result has no error code (Iron+ does).
- **Explore** marks unreachable zones as searched to avoid loops — a simplification.
- **Planner memory** is stateless per cycle apart from the last 5 goal outcomes; no episodic memory of where the target was seen.
- **Tracker** is template matching (or KCF if opencv-contrib is present); it will drift on scale change and is meant to bridge 1.5 s gaps, not track for minutes.
- **VLM prompt** is one-shot; no few-shot examples, no bbox calibration against the 450M model's actual output habits.
- **No auth** on the HTTP bridge (LAN-only assumption). werkzeug dev server; swap for waitress if it ever matters.
- **No unit tests for** the ROS adapter, vision pipeline threading, or the Flask app (the stub run is the integration test).
- **Battery voltage curve** is linear between cell empty/full — good enough to trigger `insufficient_battery`, not for accurate percent.
- Metrics/eval tooling over the JSONL log is left to you; the events are there.
