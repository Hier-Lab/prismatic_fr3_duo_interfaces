# RobotAPI Python client

`robot_motion_api` is a blocking Python API for the services provided by the
`prismatic_fr3_duo_interfaces` package. It owns a private ROS 2 node and background executor, so a normal
Python program can use it without calling `rclpy.init()` or `rclpy.spin()`.

## Build and import

Build the package and source the workspace:

```bash
cd /home/hier-tony/Projects/dual_arm_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select prismatic_fr3_duo_interfaces
source install/setup.bash
```

Other ROS packages should declare the runtime dependency:

```xml
<exec_depend>prismatic_fr3_duo_interfaces</exec_depend>
```

The API can then be imported from any Python program:

```python
from robot_motion_api import RobotAPI, RobotAPIError
```

### Use from another ROS 2 package

Add `<exec_depend>prismatic_fr3_duo_interfaces</exec_depend>` to that package's `package.xml`, import the
module normally, and run the program after sourcing the workspace. The calling package does
not need to contain custom service-client code.

### Use from a non-ROS Python program

The Python program itself does not need to be a ROS package and does not need to call
`rclpy.init()` or spin an executor. ROS 2 and this workspace must still be installed and
sourced because the API communicates with ROS services:

```python
#!/usr/bin/env python3

from robot_motion_api import RobotAPI

with RobotAPI() as robot:
    print(robot.get_current_state())
```

Run it from any directory:

```bash
source /opt/ros/humble/setup.bash
source /home/hier-tony/Projects/dual_arm_ws/install/setup.bash
python3 /path/to/my_robot_program.py
```

If the workspace is not sourced, Python will report
`ModuleNotFoundError: No module named 'robot_motion_api'`.

Run `move_group`, `robot_motion_server`, and `gripper_server` before creating the API. Using
the class as a context manager waits for all services and releases its ROS resources cleanly:

```python
from robot_motion_api import RobotAPI

with RobotAPI() as robot:
    state = robot.get_current_state()
    print(state.group, state.joints)
```

For simulation, enable the ROS clock on the interface servers and the API client:

```bash
ros2 launch prismatic_fr3_duo_interfaces prismatic_fr3_duo_interfaces.launch.py use_sim_time:=true
```

```python
with RobotAPI(use_sim_time=True) as robot:
    print(robot.get_current_state())
```

All simulation processes must receive `/clock`; otherwise time-dependent TF lookups and
MoveIt state monitoring may wait indefinitely or report stale data.

## Cartesian control

Create poses with `RobotAPI.pose(position_xyz, quaternion_xyzw)`.

Move only the left arm:

```python
with RobotAPI() as robot:
    target = RobotAPI.pose(
        position=[0.45, 0.25, 0.55],
        orientation=[0.0, 0.0, 0.0, 1.0],
    )
    result = robot.move_left(target)
    print(result.message)
```

Move only the right arm by a relative XYZ offset while preserving its orientation:

```python
with RobotAPI() as robot:
    offset = RobotAPI.pose([0.0, -0.05, 0.02], [0.0, 0.0, 0.0, 0.0])
    robot.move_right(offset, relative=True, relative_frame="end_effector")
```

Use `relative_frame="planning"` for offsets along the fixed MoveIt planning-frame axes, or
`relative_frame="end_effector"` for offsets along the current tool axes. The corresponding
numeric constants are `RobotAPI.PLANNING_FRAME` and `RobotAPI.END_EFFECTOR_FRAME`.

Move both arms in one coordinated plan:

```python
with RobotAPI(call_timeout=180.0) as robot:
    left = RobotAPI.pose([0.45, 0.30, 0.55])
    right = RobotAPI.pose([0.45, -0.30, 0.55])
    robot.move_dual(left, right, planning_group="whole_body")
```

Track the global plan through the operational-space whole-body controller:

```python
with RobotAPI(call_timeout=180.0) as robot:
    left = RobotAPI.pose([0.55, 0.25, 0.35])
    right = RobotAPI.pose([0.55, -0.25, 0.35])
    result = robot.move_l_dual(
        left,
        right,
        planning_group="whole_body",
        position_tolerance=0.01,
        orientation_tolerance=0.05,
        tracking_timeout=5.0,
        timeout=180.0,
    )
    print(result.published_samples, result.final_position_errors)
```

Unlike `move_dual()`, this path does not call MoveIt's trajectory execution action. It streams
FK-derived pose targets to `/whole_body_controller/target_pose/left` and `/right`, plus the
planned posture to `/whole_body_controller/target_joint`. The call returns only after final
measured tracking passes or the tracking timeout expires.

For custom end-effector names, call the general method:

```python
robot.move_cartesian(
    {
        "left_fr3_hand_tcp": left,
        "right_fr3_hand_tcp": right,
    },
    planning_group="dual_arm",
)
```

## Joint control

A name-to-position mapping updates only those joints:

```python
with RobotAPI() as robot:
    robot.move_joints(
        {
            "left_fr3_joint1": 0.15,
            "right_fr3_joint1": -0.15,
        },
        planning_group="dual_arm",
    )
```

Relative named-joint offsets are also supported:

```python
robot.move_joints(
    {"left_fr3_joint2": 0.1},
    planning_group="left_arm",
    relative=True,
)
```

To send every variable in MoveIt's group order, pass a list without joint names:

```python
robot.move_joints(
    [0.0, -0.4, 0.0, -2.0, 0.0, 1.6, 0.8],
    planning_group="left_arm",
)
```

Move a group to a predefined SRDF `<group_state>` without copying its joint values into
application code:

```python
with RobotAPI() as robot:
    robot.move_group_state("ready", planning_group="dual_arm")
```

The state must be defined for that exact planning group in the SRDF. If it is unknown, the
service response lists the states available for the selected group.

## Gripper control

```python
with RobotAPI() as robot:
    robot.open_gripper("left_ee")
    robot.close_gripper("right_ee")

    # Validate and execute both commands in one request.
    robot.control_grippers({"left_ee": "close", "right_ee": "open"})
```

The active MoveIt configuration must provide controllers for the `left_hand` and
`right_hand` finger joints. Otherwise the server can plan but cannot execute gripper motion.

## TF transforms

`get_transform(target_frame, source_frame)` returns the transform that expresses the source
frame in the target frame:

To visualize two absolute IK targets in RViz and request dual-arm IK, run the included
shell helper. Each quoted pose uses `x y z qx qy qz qw` order:

```bash
ros2 run prismatic_fr3_duo_interfaces publish_ik_targets.sh \
  "0.6622508 0.04624189 0.18144363 -0.08052143 0.9203639 -0.03335312 -0.38122718" \
  "0.66225074 -0.04624193 0.1814436 0.08052146 0.92036389 0.03335307 -0.3812272"
```

The parent frame defaults to `world`; supply it as a third argument when needed. Set
`USE_SIM_TIME=true` before the command when visualizing against a simulation clock.
The helper publishes both target frames, waits two seconds, calls MoveIt's `/compute_ik`
service for the `dual_arm` group, and keeps the target transforms available until Ctrl+C.

```python
with RobotAPI() as robot:
    transform = robot.get_transform("world", "left_fr3_hand_tcp", timeout=2.0)
    position, quaternion = robot.get_transform_pos_quat(
        "world", "left_fr3_hand_tcp", timeout=2.0
    )
    print(position, quaternion)
```

Publish a transform from a 4-by-4 homogeneous NumPy matrix:

```python
import numpy as np

camera_in_tool = np.eye(4)
camera_in_tool[:3, 3] = [0.0, 0.0, 0.12]

with RobotAPI() as robot:
    robot.publish_static_transform(
        "left_fr3_hand_tcp", "tool_camera", camera_in_tool
    )
```

Use `publish_dynamic_transform()` for transforms that will be updated repeatedly. The API
validates that the matrix is finite, homogeneous, and contains a proper rotation.

## Camera capture

Camera subscriptions are opt-in. The API uses approximate synchronization so each returned
color/depth pair has closely matching ROS timestamps:

```python
with RobotAPI(
    enable_camera=True,
    rgb_topic="/camera/color/image_raw",
    depth_topic="/camera/depth/image_raw",
) as robot:
    color, depth = robot.get_camera_images(wait_timeout=5.0)
    print(color.shape, depth.shape)

    color_path, depth_path = robot.save_camera_images(
        "/tmp/capture/color.png",
        "/tmp/capture/depth.png",
        wait_timeout=5.0,
    )
```

Camera support can also be enabled later:

```python
with RobotAPI() as robot:
    robot.enable_camera(
        rgb_topic="/camera/color/image_raw",
        depth_topic="/camera/aligned_depth_to_color/image_raw",
        queue_size=5,
        slop=0.1,
    )
    color, depth = robot.get_images(wait_timeout=5.0)
```

Floating-point depth images are assumed to use meters. When saving, the default
`depth_scale=1000.0` converts them to millimeters in a 16-bit PNG. Integer depth images are
saved without rescaling. Change `depth_scale` if the camera uses different units.

## Results, errors, and timeouts

Motion methods return `ServiceResult`, which contains `success` and `message`. By default a
failed server response raises `RobotAPIError`:

```python
from robot_motion_api import RobotAPI, RobotAPIError

try:
    with RobotAPI(call_timeout=180.0) as robot:
        robot.move_left(RobotAPI.pose([10.0, 0.0, 0.0]))
except RobotAPIError as error:
    print(f"Robot command failed: {error}")
```

Set `raise_on_failure=False` to handle unsuccessful responses directly:

```python
with RobotAPI(raise_on_failure=False) as robot:
    result = robot.move_left(RobotAPI.pose([10.0, 0.0, 0.0]))
    if not result:
        print(result.message)
```

Service names and default timeouts can be overridden in the constructor. Individual motion
methods also accept a `timeout=` argument. This timeout controls how long the Python client
waits for the complete service call; it does not change MoveIt's IK solver timeout. Set the
server timeout separately when launching the interfaces, for example:

```bash
ros2 launch prismatic_fr3_duo_interfaces prismatic_fr3_duo_interfaces.launch.py ik_timeout:=1.0
```

For coordinated relative motion, begin with small offsets and a zero quaternion to preserve
each tool's current orientation. `whole_body` includes the shared base axis and can solve
targets that are unreachable for the arm-only `dual_arm` group.
