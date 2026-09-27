"""Example showing the TF and synchronized camera features in RobotAPI."""

from robot_api import RobotAPI


def main() -> None:
    """Read one tool transform and save one synchronized RGB-D image pair."""
    with RobotAPI(enable_camera=True) as robot:
        position, quaternion = robot.get_transform_pos_quat(
            "world", "left_fr3_hand_tcp", timeout=2.0
        )
        print("left tool position:", position)
        print("left tool quaternion:", quaternion)

        color_path, depth_path = robot.save_camera_images(
            "/tmp/robot_capture/color.png",
            "/tmp/robot_capture/depth.png",
            wait_timeout=5.0,
        )
        print("saved color image:", color_path)
        print("saved depth image:", depth_path)


def test() -> None:
    """Test function to demonstrate RobotAPI usage."""
    # Always use the context manager (or try/finally with robot.close()).  It
    # shuts down the background ROS executor even when a command raises.
    with RobotAPI(call_timeout=5.0) as robot:
        robot.open_gripper("left_hand")

        # Example of getting the current state
        state = robot.get_current_state()
        print("Current state group:", state.group)
        print("Current state joints:", state.joints)

        # Relative poses are offsets. Select whether their axes come from the fixed
        # planning frame or from each tool; a zero quaternion preserves orientation.
        left_pose = RobotAPI.pose([0.0, 0.0, -0.1], [0.0, 0.0, 0.0, 0.0])
        right_pose = RobotAPI.pose([-0.1, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0])

        # whole_body can also move the shared base slider and is therefore more
        # likely to find a coordinated IK solution than dual_arm when needed.
        robot.move_dual(
            left_pose,
            right_pose,
            planning_group="whole_body",
            relative=True,
            relative_frame="planning",
            timeout=30.0,
        )

        robot.close_gripper("left_hand")


if __name__ == "__main__":
    # main()
    test()
