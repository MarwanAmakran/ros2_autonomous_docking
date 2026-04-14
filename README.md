# ros2_autonomous_docking

ROS 2 workspace for autonomous docking with ArUco detection.

## Docking-related nodes

- `camera_stream_publisher` publishes camera frames on `/camera/image_raw`.
- `aruco_detector` detects ArUco marker and publishes `/dock_position` as `geometry_msgs/Point`:
	- `x` = marker center x in pixels
	- `y` = marker area in pixels² (distance heuristic)
	- `z` = marker ID
- `dock_controller` runs docking state machine and publishes `/cmd_vel`.
- `cmdvel_serial_bridge` converts `/cmd_vel` to serial motor command (`D left right 1`).
- `undock_robot` publishes a one-shot trigger on `/undock_robot`.

## Docking state machine

`IDLE -> SEARCH -> ALIGN -> APPROACH -> DOCKED -> UNDOCK -> IDLE`

- `SEARCH`: rotate until target marker is visible.
- `ALIGN`: center marker in image.
- `APPROACH`: move forward slowly while correcting heading.
- `DOCKED`: stop when marker is centered and large enough (`dock_area_threshold`).
- `UNDOCK`: move backward for a short time then return to `SEARCH`.

## Topics

- Input:
	- `/camera/image_raw` (`sensor_msgs/Image`)
	- `/battery_state` (`sensor_msgs/BatteryState`, optional)
- Outputs:
	- `/cmd_vel` (`geometry_msgs/Twist`)
	- `/dock_status` (`geometry_msgs/Point`):
	  - `x` = state code
	  - `y` = estimated distance to marker (cm)
	  - `z` = heading error (normalized)
	- `/charging_state` (`geometry_msgs/Point`):
	  - `x` = charging code
	  - `y` = current battery voltage (V)
	  - `z` = alarm flag (1=low battery alarm)
	- `/undock_robot` trigger (`geometry_msgs/Point`, set `x=1.0`)
	- `/dock_robot` trigger (`geometry_msgs/Point`, set `x=1.0`)
	- `/stop_robot` emergency stop (`geometry_msgs/Point`, `x=1.0` stop, `x=0.0` release)
	- `/dock_mode` single control topic (`x=0 idle, 1 dock, 2 undock, 3 stop)

State codes on `/dock_status` (`x`):

- `1`: SEARCH
- `2`: ALIGN
- `3`: APPROACH
- `4`: DOCKED
- `5`: UNDOCK

Extra state code:

- `0`: IDLE

Charging codes on `/charging_state` (`x`):

- `0`: UNKNOWN
- `1`: DISCHARGING
- `2`: CHARGING
- `3`: LOW_BATTERY_ALARM

## Important parameters

- `target_marker_id` (default `-1`, accept any ID)
- `search_angular_speed`
- `approach_linear_speed`
- `marker_timeout_sec`
- `dock_area_threshold`
- `approach_timeout_sec`
- `undock_linear_speed`
- `undock_duration_sec`
- `command_move_sec` and `command_rest_sec` (step-by-step behavior)
- `distance_scale_cm_px` (distance estimate calibration)
- `low_battery_alarm_v` (default 6.0V)

Bridge tuning parameters (`cmdvel_serial_bridge`):

- `min_pwm_move`: minimum motor PWM so tiny commands still move the wheels
- `linear_gain`, `angular_gain`: conversion from `/cmd_vel` to motor PWM
- `cmd_timeout_sec`: fail-safe stop if no fresh command

## Typical run order

1. Start camera publisher.
2. Start `aruco_detector`.
3. Start `dock_controller`.
4. Start `cmdvel_serial_bridge`.
5. Trigger dock with `/dock_robot` (`Point.x=1.0`).
6. Trigger undock when needed with `undock_robot`.

## Quick debugging notes

- If you see very small wheel values like `D -1 1 1`, the command is too weak.
	Increase `min_pwm_move` or gains.
- Current bridge auto-fixes this with `min_pwm_move` deadband.
- If robot spins too aggressively, lower `angular_gain` in bridge or lower
	`search_angular_speed` in controller.
- If marker gets lost too often, increase `marker_timeout_sec` slightly.

## Step-by-step motion (important)

Controller now runs in steps:

1) Compute one movement command from vision/state.
2) Execute it for `command_move_sec`.
3) Stop and rest for `command_rest_sec`.
4) Re-check camera/battery and decide next step.

This avoids continuous spinning loops and gives wheels time to execute each step.

## Deliverables checklist (for your assignment)

Software done in this repo:

- ArUco detection + marker ID output
- Docking state machine (IDLE/SEARCH/ALIGN/APPROACH/DOCKED/UNDOCK)
- Charging state handling (`/battery_state` + fallback slope)
- Fail-safes (`cmd_timeout_sec`, approach/search timeout, low battery alarm)

Still to add manually (hardware/report/demo):

- Physical docking station design (mechanical guide + safe 12V pads)
- Electrical schematic and safety notes (fuse, reverse-polarity protection)
- Photos of prototype
- Live demo recording and explanation