# Robot interfaces

ROS 2 services for planning and executing single-arm, dual-arm, and gripper motions through
MoveIt. Start the corresponding server after `move_group` is running. The interface copies its
URDF, SRDF, and planning-limit parameters from that running MoveIt node, so this package has no
runtime dependency on a robot-specific MoveIt configuration package.

## Launch

After the MoveIt and controller stack is running, start both interface servers with:

```bash
ros2 launch prismatic_fr3_duo_interfaces prismatic_fr3_duo_interfaces.launch.py
```

By default, the launch connects to `/move_group`. It deliberately does not copy kinematics
parameters into the interface processes: Cartesian IK is handled by the `/compute_ik` service of
the already-running MoveIt node. Startup fails with a clear message if the remote parameter
services or required model parameters are unavailable.
Common overrides include:

```bash
ros2 launch prismatic_fr3_duo_interfaces prismatic_fr3_duo_interfaces.launch.py \
  use_sim_time:=true \
  move_group_node:=/move_group \
  default_planning_group:=whole_body \
  velocity_scale:=0.3 \
  start_gripper_server:=false
```

Run `ros2 launch prismatic_fr3_duo_interfaces prismatic_fr3_duo_interfaces.launch.py --show-args` for every available
setting. When using a namespace, also configure clients with the namespaced service paths;
set `move_group_node`, `move_group_namespace`, and `compute_ik_service` consistently. For example:

```bash
ros2 launch prismatic_fr3_duo_interfaces prismatic_fr3_duo_interfaces.launch.py \
  move_group_node:=/robot/move_group \
  move_group_namespace:=/robot \
  compute_ik_service:=/robot/compute_ik
```

The C++ executables also accept locally supplied `robot_description` and
`robot_description_semantic` parameters. When both are present, they take precedence and remote
model discovery is skipped.

## Services

| Service | Type | Purpose |
| --- | --- | --- |
| `/move_cartesian` | `prismatic_fr3_duo_interfaces/srv/Move3DPose` | One or two end-effector pose goals |
| `/move_joint` | `prismatic_fr3_duo_interfaces/srv/MoveJointPose` | Full-group or named-joint goals |
| `/move_group_state` | `prismatic_fr3_duo_interfaces/srv/MoveGroupState` | Predefined SRDF group state |
| `/move_l` | `prismatic_fr3_duo_interfaces/srv/MoveL` | Globally planned task-space tracking |
| `/get_current_state` | `std_srvs/srv/Trigger` | Joint state for the default group |
| `/control_gripper` | `prismatic_fr3_duo_interfaces/srv/GripperCommand` | Open or close one or two grippers |

Cartesian requests accept the SRDF names `left_ee` and `right_ee`, or the equivalent tip
links `left_fr3_hand_tcp` and `right_fr3_hand_tcp`. The selected planning group must contain
every requested end effector. The server asks MoveIt's `/compute_ik` service for a concrete,
collision-checked joint goal before planning; this supports multi-tip groups without relying
on OMPL's pose-goal sampler.

### Single-arm relative motion

Set `relative_frame: 0` (`PLANNING_FRAME`) to express XYZ and rotation offsets in the fixed
MoveIt planning frame. Set it to `1` (`END_EFFECTOR_FRAME`) to use the tool's current local
axes. The zero quaternion in either mode means "keep the current orientation".

```bash
ros2 service call /move_cartesian prismatic_fr3_duo_interfaces/srv/Move3DPose \
  "{planning_group: left_arm, end_effector_names: [left_ee], \
  ee_poses: [{position: {x: 0.05, y: 0.0, z: 0.0}}], \
  mode: 1, relative_frame: 0}"
```

Change `left_arm`/`left_ee` to `right_arm`/`right_ee` for a right-only request.
For example, the same command with `relative_frame: 1` moves 5 cm along the hand's local
positive X axis instead of the planning frame's positive X axis.

### Dual-arm absolute motion

`whole_body` is the configured multi-tip IK group. `dual_arm` is also accepted and is useful
for joint-space requests or when a dual-arm kinematics solver is configured.

```bash
ros2 service call /move_cartesian prismatic_fr3_duo_interfaces/srv/Move3DPose \
  "{planning_group: whole_body, end_effector_names: [left_ee, right_ee], \
  ee_poses: [
    {position: {x: 0.45, y: 0.30, z: 0.55}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}},
    {position: {x: 0.45, y: -0.30, z: 0.55}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}
  ], mode: 0}"
```

### Named-joint motion

Named-joint requests update only the listed variables. Unlisted joints retain their current
target values.

```bash
ros2 service call /move_joint prismatic_fr3_duo_interfaces/srv/MoveJointPose \
  "{planning_group: dual_arm, joint_names: [left_fr3_joint1, right_fr3_joint1], \
  joint_positions: [0.15, -0.15], mode: 0}"
```

When `joint_names` is empty, supply all group variables in MoveIt's group-variable order.
Mode `0` is absolute and mode `1` adds offsets to current joint positions.

### Predefined SRDF group state

The state name must be defined as a `<group_state>` for the selected group in the SRDF:

```bash
ros2 service call /move_group_state prismatic_fr3_duo_interfaces/srv/MoveGroupState \
  "{planning_group: dual_arm, group_state: ready}"
```

The server resolves the named state from the active robot model, plans from the current
measured state, and executes the resulting joint trajectory.

### Globally planned task-space tracking

`/move_l` obtains collision-aware IK, asks MoveIt for a time-parameterized joint trajectory,
and does not send that trajectory to a joint trajectory controller. Instead, it interpolates
`q_d(t)` at `tracking_publish_rate`, computes FK for both configured TCPs, and publishes the
resulting `PoseStamped` references to the operational-space controller. It also publishes the
matching desired Cartesian velocity as `TwistStamped` on
`/dual_arm_controller_wbc/target_velocity/left` and
`/dual_arm_controller_wbc/target_velocity/right`. The twist is computed as
`x_dot_d = J(q_d) q_dot_d`. It also publishes the joint sample, including `q_dot_d`, as the
controller's secondary posture reference. Every successful plan is also
published as `moveit_msgs/DisplayTrajectory` on `/display_planned_path` for RViz animation.
Discrete FK TCP waypoints are published as `visualization_msgs/Marker` sphere lists on
`/display_planned_waypoints/left_ee` and `/display_planned_waypoints/right_ee`.
Each target sample also publishes measured `PoseStamped` feedback on
`/dual_arm_controller_wbc/current_pose/left` and
`/dual_arm_controller_wbc/current_pose/right`. Measured TCP velocity is published as
`geometry_msgs/TwistStamped` on `/dual_arm_controller_wbc/current_velocity/left` and
`/dual_arm_controller_wbc/current_velocity/right`. Linear and angular components are computed
as `x_dot = J(q) q_dot`: `q` comes from MoveIt's current state, while measured `q_dot` is read
by joint name from `/joint_states` (configurable with `joint_state_topic`). The controller or
simulator must populate `sensor_msgs/JointState.velocity`; planned velocity is not substituted
for measured feedback. All feedback uses the same
observation timestamp and `tracking_frame`, and is published while MoveL is active.

By default, MoveL publishes at 200 Hz. Each target uses the trajectory pose at the current
elapsed ROS time, beginning near `x_d(0)`; it does not publish future trajectory poses.
Measured current-pose messages use their observation time rather than the target timestamp.
With `interpolate_velocity:=true` (default), waypoint positions and velocities are sampled with
cubic Hermite splines, matching `joint_trajectory_controller` when those interfaces are supplied.
With it disabled, position is interpolated linearly while joint and Cartesian target velocities
are explicitly zero. Both modes publish zero target velocity while holding the final point.

```bash
ros2 service call /move_l prismatic_fr3_duo_interfaces/srv/MoveL "{
  planning_group: whole_body,
  end_effector_names: [left_ee, right_ee],
  ee_poses: [
    {position: {x: 0.55, y: 0.25, z: 0.35}, orientation: {w: 1.0}},
    {position: {x: 0.55, y: -0.25, z: 0.35}, orientation: {w: 1.0}}
  ],
  mode: 0,
  position_tolerance: 0.01,
  orientation_tolerance: 0.05,
  tracking_timeout: 5.0
}"
```

The service is synchronous. `success=true` means planning succeeded, controller subscribers
were connected, every reference sample was published, and measured final TCP errors entered
the requested tolerances. The response also reports sample count, planned duration, and final
position/orientation errors. A MoveIt joint-space plan is globally collision-free but does not
guarantee a geometrically straight TCP path despite the `MoveL` service name.

Interpolation, command publication, and convergence checks use the node's ROS clock. Launch with
`use_sim_time:=true` to follow `/clock`; pausing simulation therefore pauses both motion and the
tracking timeout. The Python API call timeout remains a wall-time safety watchdog, so increase
`call_timeout` when simulation runs substantially slower than real time.

In RViz, add or enable the MoveIt `MotionPlanning` display and its `Planned Path` section; it
listens to the standard `/display_planned_path` topic. The publisher uses transient-local
durability, so the latest plan remains available when RViz starts after the service call.
To show only the end-effector waypoints, add two RViz `Marker` displays and select the left and
right marker topics. The server colors the left waypoints blue and the right waypoints orange.

Before calling the service, verify that the controller is active and its three command topics
have subscribers:

```bash
ros2 control list_controllers
ros2 topic info /whole_body_controller/target_pose/left
ros2 topic info /whole_body_controller/target_pose/right
ros2 topic info /whole_body_controller/target_joint
```

For production monitoring, also publish the controller's instantaneous task errors and active
state on a feedback topic. The current service verifies measured final FK error through MoveIt's
robot-state monitor, but it does not report maximum tracking error along the path. Use a ROS 2
action instead of a service if progress feedback or cancellation is required.

### Grippers

```bash
ros2 service call /control_gripper prismatic_fr3_duo_interfaces/srv/GripperCommand \
  "{end_effector_names: [left_ee, right_ee], commands: [open, close]}"
```

Gripper operations are validated together and executed in request order. The server uses
SRDF `open`/`close` named states when present; otherwise it uses the `open_position` and
`closed_position` parameters.

The active robot's MoveIt controller configuration must expose controllers for every configured
gripper group. Planning can succeed without those controllers, but execution cannot.

## Parameters

`robot_motion_server`:

- `move_group_node` (`/move_group`), `move_group_namespace` (empty)
- `move_group_parameter_timeout` (`10.0` wall-time seconds)
- `default_planning_group` (`whole_body`)
- `supported_planning_groups` (empty permits every group in the loaded SRDF)
- `planning_time` (`5.0`), `planning_attempts` (`10`)
- `velocity_scale` (`0.5`), `acceleration_scale` (`0.5`)
- `ik_timeout` (`0.25`), `compute_ik_service` (`/compute_ik`)
- `display_trajectory_topic` (`/display_planned_path`)
- `display_ee_waypoint_topics`, `display_ee_waypoint_size` (`0.02` m)
- `tracking_tip_links` (`[left_fr3_hand_tcp, right_fr3_hand_tcp]`)
- `tracking_pose_topics`, `tracking_velocity_topics`, `current_pose_topics`,
  `current_velocity_topics`,
  `tracking_joint_topic`, `joint_state_topic` (`/joint_states`), `tracking_frame`
  (`rail_link`)
- `tracking_publish_rate` (`200.0`)
- `interpolate_velocity` (`true`; cubic velocity feed-forward, or linear position with zero
  target velocity when disabled)
- `tracking_position_tolerance` (`0.01` m), `tracking_orientation_tolerance` (`0.05` rad)
- `tracking_timeout` (`5.0` s), `require_tracking_subscribers` (`true`)

`gripper_server`:

- `move_group_node` (`/move_group`), `move_group_namespace` (empty)
- `move_group_parameter_timeout` (`10.0` wall-time seconds)
- `gripper_groups` (`[left_hand, right_hand]`)
- `open_position` (`0.035`), `closed_position` (`0.0`)
- `planning_time` (`3.0`), `velocity_scale` (`0.5`), `acceleration_scale` (`0.5`)
