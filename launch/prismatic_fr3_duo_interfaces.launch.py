"""Launch the dual-arm motion and gripper service servers."""

from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    """Create the robot interface launch description."""
    arguments = [
        DeclareLaunchArgument(
            "namespace",
            default_value="",
            description="Optional namespace for both interface nodes and services.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use the simulation clock.",
        ),
        DeclareLaunchArgument(
            "start_gripper_server",
            default_value="true",
            description="Start the gripper service server.",
        ),
        DeclareLaunchArgument(
            "move_group_node",
            default_value="/move_group",
            description="Running MoveIt node from which robot model parameters are copied.",
        ),
        DeclareLaunchArgument(
            "move_group_namespace",
            default_value="",
            description="Namespace containing MoveIt's move_group action and services.",
        ),
        DeclareLaunchArgument(
            "move_group_parameter_timeout",
            default_value="10.0",
            description="Wall-time seconds to wait for move_group parameter services.",
        ),
        DeclareLaunchArgument(
            "default_planning_group",
            default_value="dual_arm",
            description="Planning group used when a request leaves the group empty.",
        ),
        DeclareLaunchArgument("planning_time", default_value="5.0"),
        DeclareLaunchArgument("planning_attempts", default_value="10"),
        DeclareLaunchArgument("velocity_scale", default_value="0.2"),
        DeclareLaunchArgument("acceleration_scale", default_value="0.2"),
        DeclareLaunchArgument("ik_timeout", default_value="1.0"),
        DeclareLaunchArgument(
            "compute_ik_service",
            default_value="/compute_ik",
            description="MoveIt GetPositionIK service used for Cartesian requests.",
        ),
        DeclareLaunchArgument(
            "display_trajectory_topic",
            default_value="/display_planned_path",
            description="MoveIt DisplayTrajectory topic consumed by RViz.",
        ),
        DeclareLaunchArgument(
            "display_left_ee_waypoint_topic",
            default_value="/display_planned_waypoints/left_ee",
        ),
        DeclareLaunchArgument(
            "display_right_ee_waypoint_topic",
            default_value="/display_planned_waypoints/right_ee",
        ),
        DeclareLaunchArgument("display_ee_waypoint_size", default_value="0.02"),
        DeclareLaunchArgument(
            "tracking_left_pose_topic",
            default_value="/dual_arm_controller_wbc/target_pose/left",
        ),
        DeclareLaunchArgument(
            "tracking_right_pose_topic",
            default_value="/dual_arm_controller_wbc/target_pose/right",
        ),
        DeclareLaunchArgument(
            "tracking_left_velocity_topic",
            default_value="/dual_arm_controller_wbc/target_velocity/left",
        ),
        DeclareLaunchArgument(
            "tracking_right_velocity_topic",
            default_value="/dual_arm_controller_wbc/target_velocity/right",
        ),
        DeclareLaunchArgument(
            "current_left_pose_topic",
            default_value="/dual_arm_controller_wbc/current_pose/left",
        ),
        DeclareLaunchArgument(
            "current_right_pose_topic",
            default_value="/dual_arm_controller_wbc/current_pose/right",
        ),
        DeclareLaunchArgument(
            "current_left_velocity_topic",
            default_value="/dual_arm_controller_wbc/current_velocity/left",
        ),
        DeclareLaunchArgument(
            "current_right_velocity_topic",
            default_value="/dual_arm_controller_wbc/current_velocity/right",
        ),
        DeclareLaunchArgument(
            "tracking_joint_topic",
            default_value="/dual_arm_controller_wbc/target_joint",
        ),
        DeclareLaunchArgument(
            "joint_state_topic",
            default_value="/joint_states",
            description="Measured JointState source used for current q_dot.",
        ),
        DeclareLaunchArgument("tracking_frame", default_value="rail_link"),
        DeclareLaunchArgument("tracking_publish_rate", default_value="200.0"),
        DeclareLaunchArgument(
            "interpolate_velocity",
            default_value="true",
            description=(
                "Use cubic position/velocity interpolation; false publishes zero velocity."
            ),
        ),
        DeclareLaunchArgument("tracking_position_tolerance", default_value="0.01"),
        DeclareLaunchArgument("tracking_orientation_tolerance", default_value="0.05"),
        DeclareLaunchArgument("tracking_timeout", default_value="5.0"),
        DeclareLaunchArgument("require_tracking_subscribers", default_value="true"),
        DeclareLaunchArgument("gripper_planning_time", default_value="3.0"),
        DeclareLaunchArgument("gripper_velocity_scale", default_value="0.5"),
        DeclareLaunchArgument("gripper_acceleration_scale", default_value="0.5"),
        DeclareLaunchArgument("open_position", default_value="0.035"),
        DeclareLaunchArgument("closed_position", default_value="0.0"),
    ]

    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")

    motion_parameters = [
        {
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "move_group_node": LaunchConfiguration("move_group_node"),
            "move_group_namespace": LaunchConfiguration("move_group_namespace"),
            "move_group_parameter_timeout": ParameterValue(
                LaunchConfiguration("move_group_parameter_timeout"), value_type=float
            ),
            "default_planning_group": LaunchConfiguration("default_planning_group"),
            "planning_time": ParameterValue(
                LaunchConfiguration("planning_time"), value_type=float
            ),
            "planning_attempts": ParameterValue(
                LaunchConfiguration("planning_attempts"), value_type=int
            ),
            "velocity_scale": ParameterValue(
                LaunchConfiguration("velocity_scale"), value_type=float
            ),
            "acceleration_scale": ParameterValue(
                LaunchConfiguration("acceleration_scale"), value_type=float
            ),
            "ik_timeout": ParameterValue(
                LaunchConfiguration("ik_timeout"), value_type=float
            ),
            "compute_ik_service": LaunchConfiguration("compute_ik_service"),
            "display_trajectory_topic": LaunchConfiguration(
                "display_trajectory_topic"
            ),
            # Humble treats a flat substitution list as one concatenated string.
            "display_ee_waypoint_topics": ParameterValue(
                [
                    [LaunchConfiguration("display_left_ee_waypoint_topic")],
                    [LaunchConfiguration("display_right_ee_waypoint_topic")],
                ],
                value_type=List[str],
            ),
            "display_ee_waypoint_size": ParameterValue(
                LaunchConfiguration("display_ee_waypoint_size"), value_type=float
            ),
            "tracking_tip_links": [
                "left_fr3_hand_tcp",
                "right_fr3_hand_tcp",
            ],
            # Wrap each substitution separately. In ROS 2 Humble, a flat list of
            # substitutions is treated as one concatenated string instead of a
            # string-array parameter.
            "tracking_pose_topics": ParameterValue(
                [
                    [LaunchConfiguration("tracking_left_pose_topic")],
                    [LaunchConfiguration("tracking_right_pose_topic")],
                ],
                value_type=List[str],
            ),
            "tracking_velocity_topics": ParameterValue(
                [
                    [LaunchConfiguration("tracking_left_velocity_topic")],
                    [LaunchConfiguration("tracking_right_velocity_topic")],
                ],
                value_type=List[str],
            ),
            "current_pose_topics": ParameterValue(
                [
                    [LaunchConfiguration("current_left_pose_topic")],
                    [LaunchConfiguration("current_right_pose_topic")],
                ],
                value_type=List[str],
            ),
            "current_velocity_topics": ParameterValue(
                [
                    [LaunchConfiguration("current_left_velocity_topic")],
                    [LaunchConfiguration("current_right_velocity_topic")],
                ],
                value_type=List[str],
            ),
            "tracking_joint_topic": LaunchConfiguration("tracking_joint_topic"),
            "joint_state_topic": LaunchConfiguration("joint_state_topic"),
            "tracking_frame": LaunchConfiguration("tracking_frame"),
            "tracking_publish_rate": ParameterValue(
                LaunchConfiguration("tracking_publish_rate"), value_type=float
            ),
            "interpolate_velocity": ParameterValue(
                LaunchConfiguration("interpolate_velocity"), value_type=bool
            ),
            "tracking_position_tolerance": ParameterValue(
                LaunchConfiguration("tracking_position_tolerance"), value_type=float
            ),
            "tracking_orientation_tolerance": ParameterValue(
                LaunchConfiguration("tracking_orientation_tolerance"), value_type=float
            ),
            "tracking_timeout": ParameterValue(
                LaunchConfiguration("tracking_timeout"), value_type=float
            ),
            "require_tracking_subscribers": ParameterValue(
                LaunchConfiguration("require_tracking_subscribers"), value_type=bool
            ),
        },
    ]

    gripper_parameters = [
        {
            "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
            "move_group_node": LaunchConfiguration("move_group_node"),
            "move_group_namespace": LaunchConfiguration("move_group_namespace"),
            "move_group_parameter_timeout": ParameterValue(
                LaunchConfiguration("move_group_parameter_timeout"), value_type=float
            ),
            "gripper_groups": ["left_hand", "right_hand"],
            "planning_time": ParameterValue(
                LaunchConfiguration("gripper_planning_time"), value_type=float
            ),
            "velocity_scale": ParameterValue(
                LaunchConfiguration("gripper_velocity_scale"), value_type=float
            ),
            "acceleration_scale": ParameterValue(
                LaunchConfiguration("gripper_acceleration_scale"), value_type=float
            ),
            "open_position": ParameterValue(
                LaunchConfiguration("open_position"), value_type=float
            ),
            "closed_position": ParameterValue(
                LaunchConfiguration("closed_position"), value_type=float
            ),
        },
    ]

    motion_server = Node(
        package="prismatic_fr3_duo_interfaces",
        executable="robot_motion_server",
        namespace=namespace,
        output="screen",
        parameters=motion_parameters,
    )
    gripper_server = Node(
        package="prismatic_fr3_duo_interfaces",
        executable="gripper_server",
        namespace=namespace,
        output="screen",
        parameters=gripper_parameters,
        condition=IfCondition(LaunchConfiguration("start_gripper_server")),
    )

    return LaunchDescription([*arguments, motion_server, gripper_server])
