---
name: soarm-control
description: Control the robotic arm through the OpenClaw SOARM API. Use this skill when reading current joint state, moving by joint angles, moving by XYZ coordinates, or handling SOARM robot-control requests.
---

# SOARM Control

Use the existing SOARM API to control the robotic arm directly.

## API Base URL

- Default URL: `http://127.0.0.1:8000`

## Common APIs

### Read Current State

`GET /joints`

```bash
curl -sS http://127.0.0.1:8000/joints
```

Returns:

- `joints`: current joint values
- `xyz`: current end-effector position in meters

### Move By Joint Angles

`POST /move/joints`

Parameters:

- `angles`: array of 6 values in this fixed order  
  `shoulder_pan`, `shoulder_lift`, `elbow_flex`, `wrist_flex`, `wrist_roll`, `gripper`

Notes:

- Only used when moving to a fixed position
- The first 5 joints use degrees (`deg`)
- `gripper` uses a `0-100` range

Example:

```bash
curl -sS -X POST http://127.0.0.1:8000/move/joints \
  -H 'Content-Type: application/json' \
  -d '{"angles":[0,0,0,0,0,0]}'
```

### Move By XYZ

`POST /move/xyz`

Parameters:

- `x`: target x in meters, forward/backward motion of the end effector, positive is forward
- `y`: target y in meters, left/right motion of the end effector, positive is left
- `z`: target z in meters, up/down motion of the end effector, positive is up

Notes:

 - First Choice

Example:

```bash
curl -sS -X POST http://127.0.0.1:8000/move/xyz \
  -H 'Content-Type: application/json' \
  -d '{"x":0.2,"y":0.0,"z":0.2}'
```

## Visual Servo Pick API

Default URL: `http://127.0.0.1:8001`

Start server:

```bash
python scripts/visual_servo_pick_server.py
```

### Trigger a Pick Task

`POST /pick`

Starts a full visual servo pick-and-place task asynchronously. Returns immediately.

```bash
curl -sS -X POST http://127.0.0.1:8001/pick
```

Returns:
jjjjjjjjjjj
- `ok`: true if task was accepted
- `message`: "pick task started"
- Returns 409 if a task is already running
jjjjjjjjjj
### Query Task Status

`GET /status`

```bash
curl -sS http://127.0.0.1:8001/status
```

Returns:

- `running`: whether a task is in progress
- `result`: result of the last completed task (`null` if none yet)
  - `ok`: true if task succeeded
  - `converged`: whether visual servo converged before picking
  - `error`: error message if failed
