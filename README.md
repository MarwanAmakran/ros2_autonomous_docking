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

`SEARCH -> ALIGN -> APPROACH -> DOCKED -> UNDOCK -> SEARCH`

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
	- `/dock_status` (`geometry_msgs/Point`, `x` is numeric state code)
	- `/charging_state` (`geometry_msgs/Point`, `x` is numeric charging code)

State codes on `/dock_status` (`x`):

- `1`: SEARCH
- `2`: ALIGN
- `3`: APPROACH
- `4`: DOCKED
- `5`: UNDOCK

Charging codes on `/charging_state` (`x`):

- `0`: UNKNOWN
- `1`: DISCHARGING
- `2`: CHARGING

## Important parameters

- `target_marker_id` (default `-1`, accept any ID)
- `search_angular_speed`
- `approach_linear_speed`
- `marker_timeout_sec`
- `dock_area_threshold`
- `approach_timeout_sec`
- `undock_linear_speed`
- `undock_duration_sec`

## Typical run order

1. Start camera publisher.
2. Start `aruco_detector`.
3. Start `dock_controller`.
4. Start `cmdvel_serial_bridge`.
5. Trigger undock when needed with `undock_robot`.