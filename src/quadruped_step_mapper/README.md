# Quadruped Step Mapper

`quadruped_step_mapper` is a standalone ROS 2 Humble package that converts `cmd_vel` into left/right step-length commands for a quadruped that behaves like a differential-drive platform:

- straight motion: left and right step lengths are equal
- turning: left and right step lengths are different
- in-place rotation: left and right step lengths have opposite signs

The package currently provides two C++ nodes:

- `cmd_vel_to_step_length_node`
  Publishes left/right step lengths on ROS topics for debugging or higher-level integration.
- `cmd_vel_to_serial_node`
  Converts the same left/right step lengths into a string and writes them to a serial port for the lower controller.

## Package Layout

```text
quadruped_step_mapper/
├── config/
│   ├── diff_like_quadruped.yaml
│   └── cmd_vel_to_serial.yaml
├── launch/
│   ├── cmd_vel_to_step_length.launch.py
│   └── cmd_vel_to_serial.launch.py
├── include/quadruped_step_mapper/
│   ├── cmd_vel_to_step_length_node.hpp
│   ├── cmd_vel_to_serial_node.hpp
│   └── step_length_mapper.hpp
├── src/
│   ├── cmd_vel_to_step_length_main.cpp
│   ├── cmd_vel_to_step_length_node.cpp
│   ├── cmd_vel_to_serial_main.cpp
│   ├── cmd_vel_to_serial_node.cpp
│   └── step_length_mapper.cpp
├── CMakeLists.txt
└── package.xml
```

## Supported Environment

- Ubuntu 22.04
- ROS 2 Humble
- `ament_cmake`
- C++14

## Dependencies

This package itself only depends on:

- `rclcpp`
- `geometry_msgs`
- `launch`
- `launch_ros`

For a clean machine, the practical install set to compile and run this package is:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-humble-rclcpp \
  ros-humble-geometry-msgs \
  ros-humble-launch \
  ros-humble-launch-ros
```

If `rosdep` has not been initialized yet:

```bash
sudo rosdep init
rosdep update
```

Then, from the workspace root:

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Notes:

- The serial node uses Linux `termios` directly. No extra third-party serial library is required.
- If you already have `ros-humble-desktop` or `ros-humble-ros-base` installed, most of the ROS dependencies above may already be present.

## Build

From the workspace root:

```bash
cd /home/ubuntu/dog_localize
source /opt/ros/humble/setup.bash
colcon build --packages-select quadruped_step_mapper
source install/setup.bash
```

This package has already been verified in this workspace with:

```bash
colcon build --packages-select quadruped_step_mapper
```

## Motion Model

The package assumes a fixed gait step frequency:

```text
T_step = 1 / step_frequency_hz
```

The conversion from `cmd_vel` to left/right step length is:

```text
left_step_length  = (vx - b * wz) * T_step
right_step_length = (vx + b * wz) * T_step
step_yaw          = wz * T_step
```

where:

- `vx` is `cmd_vel.linear.x`
- `wz` is `cmd_vel.angular.z`
- `b` is `stance_half_width_m`

Behavior:

- `wz = 0`: left and right are equal
- `wz > 0`: left turn
- `wz < 0`: right turn
- `vx = 0`, `wz != 0`: in-place rotation

The mapper also clamps and scales the result using:

- `max_linear_speed_mps`
- `max_angular_speed_rps`
- `max_step_length_m`
- `max_step_yaw_rad`
- deadband thresholds for very small `vx` and `wz`

## Node 1: `cmd_vel_to_step_length_node`

### Purpose

This node is useful when you want to inspect or debug the step-length conversion in ROS before touching the serial protocol.

### Input

- `/cmd_vel` by default
  Type: `geometry_msgs/msg/Twist`

### Outputs

- `/step_length_cmd`
  Type: `geometry_msgs/msg/Vector3Stamped`
  - `vector.x`: left step length in meters per step
  - `vector.y`: right step length in meters per step
  - `vector.z`: step yaw in radians per step

- `/step_length_twist`
  Type: `geometry_msgs/msg/TwistStamped`
  - `twist.linear.x`: average forward step length in meters per step
  - `twist.angular.z`: step yaw in radians per step

### Run

```bash
cd /home/ubuntu/dog_localize
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch quadruped_step_mapper cmd_vel_to_step_length.launch.py
```

Override the config file if needed:

```bash
ros2 launch quadruped_step_mapper cmd_vel_to_step_length.launch.py \
  config:=/absolute/path/to/your.yaml
```

Debug note:

- if you test with `ros2 topic pub --once /cmd_vel ...`, a very short timeout can make `/step_length_cmd` and `/step_length_twist` fall back to zero before `ros2 topic echo` finishes discovery
- the default debug config for `cmd_vel_to_step_length_node` now uses `cmd_vel_timeout_sec: 2.0`
- for the most stable test, still prefer a continuous publisher such as `ros2 topic pub -r 10 /cmd_vel ...`

### Default Config

See:

- [config/diff_like_quadruped.yaml](./config/diff_like_quadruped.yaml)

## Node 2: `cmd_vel_to_serial_node`

### Purpose

This node subscribes to `/cmd_vel`, converts it into left/right step lengths, normalizes them into the range `-1.0..1.0`, and writes them to a serial port as a string.

### Input

- `/cmd_vel`
  Type: `geometry_msgs/msg/Twist`
- `/dog_mode_current`
  Type: `std_msgs/msg/Int32`
  Runtime gait/motion mode selected by the upper-layer manager

### Status Outputs

- `/dog_mode_feedback/current`
  Type: `std_msgs/msg/Int32`
  Echo of the mode currently held by the serial bridge
- `/motion_bridge/serial_connected`
  Type: `std_msgs/msg/Bool`
  Whether the serial device is currently open

### Serial Output

Default string format:

```text
[mode,left_step,right_step]
```

Example:

```text
[0,-0.620,0.620]
```

Meaning:

- `mode` is the runtime mode field. Default startup value is `0`, and it can be updated
  dynamically by subscribing to `/dog_mode_current`
- `left_step` and `right_step` are normalized signed step-length commands
- both step values are clamped to the closed interval `-1.0..1.0`

Important:

- positive value means forward on that side
- negative value means backward on that side
- this keeps backward motion and in-place rotation direction in the serial protocol

### Run

```bash
cd /home/ubuntu/dog_localize
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch quadruped_step_mapper cmd_vel_to_serial.launch.py
```

Override the config file if needed:

```bash
ros2 launch quadruped_step_mapper cmd_vel_to_serial.launch.py \
  config:=/absolute/path/to/your.yaml
```

Run directly without launch:

```bash
ros2 run quadruped_step_mapper cmd_vel_to_serial_node \
  --ros-args \
  --params-file /home/ubuntu/dog_localize/src/quadruped_step_mapper/config/cmd_vel_to_serial.yaml
```

### Default Config

See:

- [config/cmd_vel_to_serial.yaml](./config/cmd_vel_to_serial.yaml)

### Important Serial Parameters

- `serial_device`
  Example: `/dev/ttyUSB0`
- `baud_rate`
  Supported in code: `9600`, `19200`, `38400`, `57600`, `115200`, `230400`
- `transmit_rate_hz`
- `reconnect_interval_sec`
- `mode`
- `mode_topic`
- `mode_echo_topic`
- `serial_connected_topic`
- `fixed_step_override_enabled`
- `fixed_step_override_mode`
- `fixed_step_override_left_norm`
- `fixed_step_override_right_norm`
- `fixed_step_override_modes`
- `fixed_step_override_left_norms`
- `fixed_step_override_right_norms`
- `output_precision`
- `message_prefix`
- `field_separator`
- `message_suffix`

Fixed-step override notes:

- Legacy single-mode parameters are still supported:
  `fixed_step_override_mode`, `fixed_step_override_left_norm`, `fixed_step_override_right_norm`
- For multiple fixed gait mappings, set the three array parameters with matching lengths:
  `fixed_step_override_modes`, `fixed_step_override_left_norms`, `fixed_step_override_right_norms`
- Runtime `/serial_step_override` still has higher priority than these fixed mode mappings

## Most Important Parameters to Tune First

These are the first parameters that usually need real-robot tuning:

- `step_frequency_hz`
  This defines the time base of the conversion. If this is wrong, all computed step lengths are wrong.
- `stance_half_width_m`
  This controls how strongly `wz` changes the left/right step difference. If this is wrong, turning will feel too weak or too strong.
- `max_step_length_m`
  This must match the real reachable step-length envelope of the gait.
- `max_linear_speed_mps`
  This should reflect the real speed range the robot can track.
- `max_angular_speed_rps`
  This should reflect the real yaw-rate range the robot can track.
- `min_effective_linear_speed_mps`
  If too low, the robot jitters at near-zero commands.
- `min_effective_angular_speed_rps`
  If too low, the robot jitters during tiny turn commands.
- `mode`
  This is the default startup mode before `/dog_mode_current` updates arrive.

## Debugging Expected Speed vs Real Speed Mismatch

If I need to debug why the robot’s actual speed does not match the expected speed from `cmd_vel`, I need the following data.

### Required ROS Topics

Record these topics with timestamps:

- `/cmd_vel`
  The original velocity command
- `/step_length_cmd`
  The ROS-side converted left/right step lengths
- `/step_length_twist`
  The ROS-side average forward step length and step yaw
- `/Odometry`
  Or your actual robot odometry topic, if different
- Any lower-controller feedback topic that reports:
  - actual left step command
  - actual right step command
  - current gait mode
  - step frequency
  - controller state or fault code

If your robot publishes IMU data:

- IMU topic
  To see slips, oscillations, or yaw-rate mismatch

If your lower controller returns wheel-like or side-like speed estimates:

- measured left-side speed
- measured right-side speed

### Required Serial Data

For the serial path, I also need:

- the exact transmitted serial strings
- the exact serial strings received or acknowledged by the lower controller
- timestamps for both TX and RX

If possible, log:

- raw TX bytes
- raw RX bytes
- parse success/failure counts

### Required Parameters

I need the exact runtime values of:

- `step_frequency_hz`
- `stance_half_width_m`
- `max_step_length_m`
- `max_step_yaw_rad`
- `max_linear_speed_mps`
- `max_angular_speed_rps`
- `min_effective_linear_speed_mps`
- `min_effective_angular_speed_rps`
- serial formatting parameters

### Required Robot-Side Hardware Data

If the robot moves slower or faster than expected, I also need:

- battery voltage during motion
- battery current if available
- motor driver fault flags
- gait mode
- whether the robot is on flat ground or slippery ground
- whether the robot is carrying payload
- whether the robot is rotating in place or translating

### Strongly Recommended Ground-Truth Data

For a real mismatch investigation, these help a lot:

- a video synchronized with ROS bag time
- a measured travel distance over a known time interval
- a measured yaw angle change over a known time interval
- surface type
  Example: tile, concrete, carpet, rubber mat

### Minimum Useful Debug Bundle

If you want the shortest useful dataset, give me:

1. A rosbag with `/cmd_vel`, `/step_length_cmd`, `/Odometry`
2. A log of the transmitted serial strings
3. The exact parameter YAML you used
4. A short video of the robot motion
5. Battery voltage during the test

## Example Debug Commands

Record the main ROS topics:

```bash
ros2 bag record /cmd_vel /step_length_cmd /step_length_twist /Odometry
```

Watch the step-length output:

```bash
ros2 topic echo /step_length_cmd
```

Watch the original command:

```bash
ros2 topic echo /cmd_vel
```

## Common Failure Cases

- `ros2 launch` fails with a log-directory permission error
  In restricted environments, set:

```bash
export ROS_LOG_DIR=/tmp/ros_logs
```

- serial node starts but cannot open the device
  Check:
  - the device path is correct
  - your user has permission to open the serial device
  - the lower controller is actually connected

- step lengths look correct in ROS but real speed is wrong
  Usually this means:
  - `step_frequency_hz` is wrong
  - lower controller is applying a different gait frequency
  - the normalized `-1.0..1.0` scale does not match the lower controller expectation
  - the lower controller interprets the sign convention differently
  - real robot traction or payload differs from assumptions

## Installed Executables

After build, the package installs:

- `cmd_vel_to_step_length_node`
- `cmd_vel_to_serial_node`

## Installed Launch Files

- `cmd_vel_to_step_length.launch.py`
- `cmd_vel_to_serial.launch.py`

## Installed Config Files

- `diff_like_quadruped.yaml`
- `cmd_vel_to_serial.yaml`
