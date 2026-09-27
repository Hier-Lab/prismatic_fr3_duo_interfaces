"""
Synchronous Python client for the prismatic_fr3_duo_interfaces ROS 2 services.

The public :class:`RobotAPI` owns an isolated ROS context, node, and background
executor. Applications therefore do not need to call ``rclpy.spin()`` to use
the blocking convenience methods in this module.
"""

from __future__ import annotations

import atexit
from dataclasses import dataclass
import math
from pathlib import Path
import threading
import time
from typing import Any, Dict, Mapping, Optional, Sequence, Tuple

from geometry_msgs.msg import Pose, TransformStamped
import rclpy
from rclpy.context import Context
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_srvs.srv import Trigger
import tf2_ros

from prismatic_fr3_duo_interfaces.srv import (
    GripperCommand,
    Move3DPose,
    MoveGroupState,
    MoveJointPose,
    MoveL,
)


class RobotAPIError(RuntimeError):
    """Base exception raised by RobotAPI."""


class ServiceUnavailableError(RobotAPIError):
    """Raised when a required robot service is not available."""


class ServiceCallTimeoutError(RobotAPIError):
    """Raised when a service does not respond before the configured timeout."""


@dataclass(frozen=True)
class ServiceResult:
    """Result returned by a motion or gripper command."""

    success: bool
    message: str

    def __bool__(self) -> bool:
        """Allow ``if result:`` as a shorthand for ``if result.success:``."""
        return self.success


@dataclass(frozen=True)
class RobotStateResult:
    """Current state reported by the server's default planning group."""

    group: str
    joints: Mapping[str, float]
    message: str


@dataclass(frozen=True)
class MoveLResult:
    """Planning, publication, and final task-space tracking result."""

    success: bool
    message: str
    published_samples: int
    planned_duration: float
    final_position_errors: Tuple[float, ...]
    final_orientation_errors: Tuple[float, ...]

    def __bool__(self) -> bool:
        """Allow direct truth testing of final tracking success."""
        return self.success


class RobotAPI:
    """
    Provide a blocking, thread-safe API for the robot service servers.

    Use this class as a context manager whenever possible::

        with RobotAPI() as robot:
            robot.move_left(RobotAPI.pose([0.4, 0.2, 0.5]))

    The API serializes calls because the server executes one robot command at a
    time. It is safe to call methods from different application threads.
    """

    ABSOLUTE = Move3DPose.Request.ABSOLUTE
    RELATIVE = Move3DPose.Request.RELATIVE
    PLANNING_FRAME = Move3DPose.Request.PLANNING_FRAME
    END_EFFECTOR_FRAME = Move3DPose.Request.END_EFFECTOR_FRAME

    def __init__(
        self,
        *,
        node_name: str = "robot_api_client",
        cartesian_service: str = "/move_cartesian",
        joint_service: str = "/move_joint",
        group_state_service: str = "/move_group_state",
        move_l_service: str = "/move_l",
        gripper_service: str = "/control_gripper",
        state_service: str = "/get_current_state",
        service_wait_timeout: float = 5.0,
        call_timeout: float = 120.0,
        raise_on_failure: bool = True,
        use_sim_time: bool = False,
        enable_camera: bool = False,
        rgb_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/depth/image_raw",
        rgb_compressed: Optional[bool] = None,
        ros_args: Optional[Sequence[str]] = None,
    ) -> None:
        """
        Create a client and start its private ROS executor.

        :param node_name: Name of the internal ROS node.
        :param cartesian_service: Cartesian motion service name.
        :param joint_service: Joint motion service name.
        :param group_state_service: Predefined SRDF group-state service name.
        :param move_l_service: Globally planned task-space tracking service name.
        :param gripper_service: Gripper command service name.
        :param state_service: Current-state service name.
        :param service_wait_timeout: Seconds to wait for a service to appear.
        :param call_timeout: Default seconds to wait for planning and execution.
        :param raise_on_failure: Raise RobotAPIError for a failed server response.
        :param use_sim_time: Use the ROS `/clock` topic instead of the system clock.
        :param enable_camera: Start synchronized RGB-D subscriptions immediately.
        :param rgb_topic: Color image topic used when camera support is enabled.
        :param depth_topic: Depth image topic used when camera support is enabled.
        :param rgb_compressed: Whether the color topic uses ``CompressedImage``.
            ``None`` infers this from the standard ``/compressed`` topic suffix.
        :param ros_args: Optional ROS arguments for the private context.

        """
        if service_wait_timeout <= 0.0:
            raise ValueError("service_wait_timeout must be greater than zero")
        if call_timeout <= 0.0:
            raise ValueError("call_timeout must be greater than zero")
        if not isinstance(use_sim_time, bool):
            raise TypeError("use_sim_time must be a bool")

        self._service_wait_timeout = float(service_wait_timeout)
        self._call_timeout = float(call_timeout)
        self._raise_on_failure = raise_on_failure
        self._use_sim_time = use_sim_time
        self._call_lock = threading.Lock()
        self._closed = False

        # A private context avoids interfering with an application's own rclpy
        # initialization, executor, or shutdown lifecycle. Set use_sim_time while
        # creating the node so every TF and message timestamp uses one clock.
        self._context = Context()
        rclpy.init(args=list(ros_args) if ros_args is not None else None, context=self._context)
        self._node: Node = rclpy.create_node(
            node_name,
            context=self._context,
            parameter_overrides=[Parameter("use_sim_time", value=use_sim_time)],
            automatically_declare_parameters_from_overrides=True,
        )
        self._executor = MultiThreadedExecutor(num_threads=2, context=self._context)
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin,
            name=f"{node_name}_executor",
            daemon=True,
        )
        self._spin_thread.start()

        # Register a last-resort shutdown hook for scripts that let an exception
        # escape before calling close().  A live rclpy executor thread during
        # interpreter teardown can otherwise cause a native process abort.
        # Context-manager cleanup remains preferred because it happens promptly.
        self._atexit_callback = self.close
        atexit.register(self._atexit_callback)

        self._cartesian_client = self._node.create_client(Move3DPose, cartesian_service)
        self._joint_client = self._node.create_client(MoveJointPose, joint_service)
        self._group_state_client = self._node.create_client(
            MoveGroupState, group_state_service
        )
        self._move_l_client = self._node.create_client(MoveL, move_l_service)
        self._state_client = self._node.create_client(Trigger, state_service)
        self._gripper_client = self._node.create_client(GripperCommand, gripper_service)

        # TF uses the API node's background executor, so lookup calls do not require
        # application code to spin a second node.
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self._node, spin_thread=False
        )
        self._static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self._node)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self._node)

        # Camera dependencies and subscriptions are loaded lazily. Motion-only users
        # therefore do not pay the subscription or OpenCV initialization cost.
        self._camera_lock = threading.Lock()
        self._camera_event = threading.Event()
        self._camera_enabled = False
        self._rgb_image: Optional[Any] = None
        self._depth_image: Optional[Any] = None
        self._rgb_subscriber = None
        self._depth_subscriber = None
        self._image_synchronizer = None
        self._cv_bridge = None
        self._h264_decoder = None
        self._rgb_encoding = "bgr8"
        self._depth_encoding = "passthrough"
        self._rgb_compressed = False
        if enable_camera:
            self.enable_camera(
                rgb_topic=rgb_topic,
                depth_topic=depth_topic,
                rgb_compressed=rgb_compressed,
            )

    @staticmethod
    def pose(
        position: Sequence[float],
        orientation: Sequence[float] = (0.0, 0.0, 0.0, 1.0),
    ) -> Pose:
        """
        Build a geometry Pose from XYZ and XYZW sequences.

        For relative Cartesian commands, pass ``(0, 0, 0, 0)`` as the
        orientation to preserve the current end-effector orientation.
        """
        if len(position) != 3:
            raise ValueError("position must contain [x, y, z]")
        if len(orientation) != 4:
            raise ValueError("orientation must contain [x, y, z, w]")

        values = tuple(float(value) for value in (*position, *orientation))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("pose values must all be finite")

        result = Pose()
        result.position.x, result.position.y, result.position.z = values[:3]
        (
            result.orientation.x,
            result.orientation.y,
            result.orientation.z,
            result.orientation.w,
        ) = values[3:]
        return result

    @classmethod
    def _relative_frame_value(cls, relative_frame: str | int) -> int:
        """Convert a friendly relative-coordinate frame name to its service value."""
        if isinstance(relative_frame, str):
            normalized = relative_frame.strip().lower().replace("-", "_")
            if normalized in {"planning", "planning_frame", "reference", "reference_frame"}:
                return cls.PLANNING_FRAME
            if normalized in {"end_effector", "end_effector_frame", "tool", "tool_frame", "ee"}:
                return cls.END_EFFECTOR_FRAME
        elif not isinstance(relative_frame, bool) and relative_frame in {
            cls.PLANNING_FRAME,
            cls.END_EFFECTOR_FRAME,
        }:
            return int(relative_frame)
        raise ValueError(
            "relative_frame must be 'planning', 'end_effector', "
            "RobotAPI.PLANNING_FRAME, or RobotAPI.END_EFFECTOR_FRAME"
        )

    def move_cartesian(
        self,
        targets: Mapping[str, Pose],
        *,
        planning_group: str = "",
        relative: bool = False,
        relative_frame: str | int = "planning",
        timeout: Optional[float] = None,
    ) -> ServiceResult:
        """
        Move one or more end effectors to Cartesian poses.

        :param targets: Mapping from semantic end-effector name or tip link to Pose.
        :param planning_group: MoveIt group, such as ``left_arm``, ``dual_arm``,
            or ``whole_body``. An empty value uses the server default.
        :param relative: Interpret each pose as an offset from its current pose.
        :param relative_frame: Express relative position and orientation offsets in
            the fixed MoveIt planning frame or each end effector's current frame.
            Accepted names are ``"planning"`` and ``"end_effector"``.
        :param timeout: Optional per-call timeout in seconds.

        """
        if not targets:
            raise ValueError("targets must contain at least one end effector")
        if not all(isinstance(target, Pose) for target in targets.values()):
            raise TypeError("every Cartesian target must be a geometry_msgs.msg.Pose")

        request = Move3DPose.Request()
        request.planning_group = planning_group
        request.end_effector_names = list(targets.keys())
        request.ee_poses = list(targets.values())
        request.mode = request.RELATIVE if relative else request.ABSOLUTE
        request.relative_frame = self._relative_frame_value(relative_frame)
        return self._call(self._cartesian_client, request, timeout)

    def move_left(
        self,
        target: Pose,
        *,
        relative: bool = False,
        relative_frame: str | int = "planning",
        planning_group: str = "left_arm",
        timeout: Optional[float] = None,
    ) -> ServiceResult:
        """Move only the left end effector."""
        return self.move_cartesian(
            {"left_ee": target},
            planning_group=planning_group,
            relative=relative,
            relative_frame=relative_frame,
            timeout=timeout,
        )

    def move_right(
        self,
        target: Pose,
        *,
        relative: bool = False,
        relative_frame: str | int = "planning",
        planning_group: str = "right_arm",
        timeout: Optional[float] = None,
    ) -> ServiceResult:
        """Move only the right end effector."""
        return self.move_cartesian(
            {"right_ee": target},
            planning_group=planning_group,
            relative=relative,
            relative_frame=relative_frame,
            timeout=timeout,
        )

    def move_dual(
        self,
        left_target: Pose,
        right_target: Pose,
        *,
        relative: bool = False,
        relative_frame: str | int = "planning",
        planning_group: str = "whole_body",
        timeout: Optional[float] = None,
    ) -> ServiceResult:
        """Move both end effectors in one coordinated plan."""
        return self.move_cartesian(
            {"left_ee": left_target, "right_ee": right_target},
            planning_group=planning_group,
            relative=relative,
            relative_frame=relative_frame,
            timeout=timeout,
        )

    def move_l(
        self,
        targets: Mapping[str, Pose],
        *,
        planning_group: str = "whole_body",
        relative: bool = False,
        relative_frame: str | int = "planning",
        position_tolerance: float = 0.0,
        orientation_tolerance: float = 0.0,
        tracking_timeout: float = 0.0,
        timeout: Optional[float] = None,
    ) -> MoveLResult:
        """Plan globally, stream FK pose references, and verify final tracking."""
        if not targets:
            raise ValueError("targets must contain at least one end effector")
        if not all(isinstance(target, Pose) for target in targets.values()):
            raise TypeError("every MoveL target must be a geometry_msgs.msg.Pose")
        values = (position_tolerance, orientation_tolerance, tracking_timeout)
        if not all(math.isfinite(value) and value >= 0.0 for value in values):
            raise ValueError(
                "MoveL tolerances and tracking_timeout must be finite and nonnegative"
            )

        request = MoveL.Request()
        request.planning_group = planning_group
        request.end_effector_names = list(targets.keys())
        request.ee_poses = list(targets.values())
        request.mode = request.RELATIVE if relative else request.ABSOLUTE
        request.relative_frame = self._relative_frame_value(relative_frame)
        request.position_tolerance = float(position_tolerance)
        request.orientation_tolerance = float(orientation_tolerance)
        request.tracking_timeout = float(tracking_timeout)
        response = self._call_response(self._move_l_client, request, timeout)
        return MoveLResult(
            success=bool(response.success),
            message=str(response.message),
            published_samples=int(response.published_samples),
            planned_duration=float(response.planned_duration),
            final_position_errors=tuple(response.final_position_errors),
            final_orientation_errors=tuple(response.final_orientation_errors),
        )

    def move_l_dual(
        self,
        left_target: Pose,
        right_target: Pose,
        **kwargs,
    ) -> MoveLResult:
        """Track coordinated left and right targets through :meth:`move_l`."""
        return self.move_l(
            {"left_ee": left_target, "right_ee": right_target}, **kwargs
        )

    def move_joints(
        self,
        positions: Sequence[float] | Mapping[str, float],
        *,
        planning_group: str = "",
        joint_names: Optional[Sequence[str]] = None,
        relative: bool = False,
        timeout: Optional[float] = None,
    ) -> ServiceResult:
        """
        Move a complete planning group or a named subset of its joints.

        A mapping is the safest form because it explicitly associates names and
        values. When a sequence is used without ``joint_names``, values must be
        in MoveIt's variable order for the selected group.
        """
        if isinstance(positions, Mapping):
            if joint_names is not None:
                raise ValueError("joint_names must be omitted when positions is a mapping")
            names = list(positions.keys())
            values = [float(value) for value in positions.values()]
        else:
            names = list(joint_names) if joint_names is not None else []
            values = [float(value) for value in positions]

        if not values:
            raise ValueError("positions must not be empty")
        if names and len(names) != len(values):
            raise ValueError("joint_names and positions must have equal length")
        if not all(math.isfinite(value) for value in values):
            raise ValueError("joint positions must all be finite")

        request = MoveJointPose.Request()
        request.planning_group = planning_group
        request.joint_names = names
        request.joint_positions = values
        request.mode = request.RELATIVE if relative else request.ABSOLUTE
        return self._call(self._joint_client, request, timeout)

    def move_joint(
        self,
        positions: Sequence[float] | Mapping[str, float],
        **kwargs,
    ) -> ServiceResult:
        """Alias for :meth:`move_joints`."""
        return self.move_joints(positions, **kwargs)

    def move_group_state(
        self,
        group_state: str,
        *,
        planning_group: str = "",
        timeout: Optional[float] = None,
    ) -> ServiceResult:
        """Move a planning group to a predefined SRDF ``<group_state>``."""
        if not group_state:
            raise ValueError("group_state must not be empty")

        request = MoveGroupState.Request()
        request.planning_group = planning_group
        request.group_state = group_state
        return self._call(self._group_state_client, request, timeout)

    def control_grippers(
        self,
        commands: Mapping[str, str],
        *,
        timeout: Optional[float] = None,
    ) -> ServiceResult:
        """
        Open or close one or more grippers.

        Example: ``{"left_ee": "open", "right_ee": "close"}``.
        """
        if not commands:
            raise ValueError("commands must contain at least one gripper")

        normalized = {name: command.lower() for name, command in commands.items()}
        invalid = [command for command in normalized.values() if command not in {"open", "close"}]
        if invalid:
            raise ValueError("gripper commands must be 'open' or 'close'")

        request = GripperCommand.Request()
        request.end_effector_names = list(normalized.keys())
        request.commands = list(normalized.values())
        return self._call(self._gripper_client, request, timeout)

    def control_gripper(
        self,
        end_effector_name: str,
        command: str,
        *,
        timeout: Optional[float] = None,
    ) -> ServiceResult:
        """Control one gripper."""
        return self.control_grippers({end_effector_name: command}, timeout=timeout)

    def open_gripper(
        self, end_effector_name: str, *, timeout: Optional[float] = None
    ) -> ServiceResult:
        """Open one gripper."""
        return self.control_gripper(end_effector_name, "open", timeout=timeout)

    def close_gripper(
        self, end_effector_name: str, *, timeout: Optional[float] = None
    ) -> ServiceResult:
        """Close one gripper."""
        return self.control_gripper(end_effector_name, "close", timeout=timeout)

    def get_current_state(self, *, timeout: Optional[float] = None) -> RobotStateResult:
        """Return the current joints reported for the server's default group."""
        result = self._call(self._state_client, Trigger.Request(), timeout)
        group, joints = self._parse_state_message(result.message)
        return RobotStateResult(group=group, joints=joints, message=result.message)

    def get_transform(
        self,
        target_frame: str,
        source_frame: str,
        *,
        timeout: float = 1.0,
    ) -> TransformStamped:
        """
        Return the transform that expresses ``source_frame`` in ``target_frame``.

        :param target_frame: Coordinate frame in which to express the result.
        :param source_frame: Coordinate frame to transform from.
        :param timeout: Maximum seconds to wait for TF data.

        """
        if not target_frame or not source_frame:
            raise ValueError("target_frame and source_frame must not be empty")
        if timeout <= 0.0:
            raise ValueError("timeout must be greater than zero")
        try:
            return self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
                timeout=Duration(seconds=float(timeout)),
            )
        except tf2_ros.TransformException as exception:
            raise RobotAPIError(
                f"unable to transform '{source_frame}' into '{target_frame}': {exception}"
            ) from exception

    def get_transform_pos_quat(
        self,
        target_frame: str,
        source_frame: str,
        *,
        timeout: float = 1.0,
    ) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
        """Return a TF transform as ``(XYZ, quaternion XYZW)`` tuples."""
        transform = self.get_transform(target_frame, source_frame, timeout=timeout)
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            (translation.x, translation.y, translation.z),
            (rotation.x, rotation.y, rotation.z, rotation.w),
        )

    def publish_static_transform(
        self,
        parent_frame: str,
        child_frame: str,
        transform_matrix: Sequence[Sequence[float]],
    ) -> TransformStamped:
        """Publish a static transform represented by a 4-by-4 homogeneous matrix."""
        transform = self._transform_from_matrix(
            parent_frame, child_frame, transform_matrix
        )
        self._static_tf_broadcaster.sendTransform(transform)
        return transform

    def publish_dynamic_transform(
        self,
        parent_frame: str,
        child_frame: str,
        transform_matrix: Sequence[Sequence[float]],
    ) -> TransformStamped:
        """Publish a dynamic transform represented by a 4-by-4 homogeneous matrix."""
        transform = self._transform_from_matrix(
            parent_frame, child_frame, transform_matrix
        )
        self._tf_broadcaster.sendTransform(transform)
        return transform

    def enable_camera(
        self,
        *,
        rgb_topic: str = "/camera/color/image_raw",
        depth_topic: str = "/camera/depth/image_raw",
        rgb_compressed: Optional[bool] = None,
        rgb_encoding: str = "bgr8",
        depth_encoding: str = "passthrough",
        queue_size: int = 5,
        slop: float = 0.1,
    ) -> None:
        """
        Subscribe to approximately synchronized color and depth images.

        Call this once before :meth:`get_camera_images` unless camera support was
        enabled in the constructor. Both raw ``Image`` and compressed
        ``CompressedImage`` color topics are supported. The latest synchronized
        pair is retained.
        """
        if self._camera_enabled:
            raise RobotAPIError("camera subscriptions are already enabled")
        if not rgb_topic or not depth_topic:
            raise ValueError("rgb_topic and depth_topic must not be empty")
        if queue_size <= 0:
            raise ValueError("queue_size must be greater than zero")
        if slop < 0.0:
            raise ValueError("slop must not be negative")
        if rgb_compressed is not None and not isinstance(rgb_compressed, bool):
            raise TypeError("rgb_compressed must be a bool or None")

        try:
            from cv_bridge import CvBridge
            from message_filters import ApproximateTimeSynchronizer, Subscriber
            from sensor_msgs.msg import CompressedImage, Image
        except ImportError as exception:
            raise RobotAPIError(
                "camera support requires cv_bridge, message_filters, and sensor_msgs"
            ) from exception

        self._rgb_encoding = rgb_encoding
        self._depth_encoding = depth_encoding
        self._rgb_compressed = (
            rgb_topic.rstrip("/").endswith("/compressed")
            if rgb_compressed is None
            else rgb_compressed
        )
        self._cv_bridge = CvBridge()
        rgb_message_type = CompressedImage if self._rgb_compressed else Image
        self._rgb_subscriber = Subscriber(self._node, rgb_message_type, rgb_topic)
        self._depth_subscriber = Subscriber(self._node, Image, depth_topic)
        self._image_synchronizer = ApproximateTimeSynchronizer(
            [self._rgb_subscriber, self._depth_subscriber],
            queue_size=queue_size,
            slop=slop,
        )
        self._image_synchronizer.registerCallback(self._camera_callback)
        self._camera_enabled = True

    def get_camera_images(
        self, *, wait_timeout: Optional[float] = None
    ) -> Tuple[Optional[Any], Optional[Any]]:
        """
        Return copies of the latest synchronized ``(color, depth)`` arrays.

        When ``wait_timeout`` is provided, wait that many seconds for the first
        synchronized pair. Without it, return ``(None, None)`` if no pair exists.
        """
        if not self._camera_enabled:
            raise RobotAPIError("camera is disabled; call enable_camera() first")
        if wait_timeout is not None:
            if wait_timeout <= 0.0:
                raise ValueError("wait_timeout must be greater than zero")
            if not self._camera_event.wait(float(wait_timeout)):
                raise ServiceCallTimeoutError(
                    f"no synchronized camera images received within {wait_timeout:.1f} seconds"
                )

        with self._camera_lock:
            if self._rgb_image is None or self._depth_image is None:
                return None, None
            return self._rgb_image.copy(), self._depth_image.copy()

    def get_images(
        self, *, wait_timeout: Optional[float] = None
    ) -> Tuple[Optional[Any], Optional[Any]]:
        """Alias for :meth:`get_camera_images`."""
        return self.get_camera_images(wait_timeout=wait_timeout)

    def save_camera_images(
        self,
        color_image_path: str,
        depth_image_path: str,
        *,
        wait_timeout: Optional[float] = None,
        depth_scale: float = 1000.0,
        create_directories: bool = True,
    ) -> Tuple[Path, Path]:
        """
        Save the latest synchronized color and depth images.

        Floating-point depth is assumed to be in meters and is converted to
        unsigned 16-bit values using ``depth_scale`` (1000 gives millimeters).
        Integer depth images are written without rescaling.
        """
        if depth_scale <= 0.0 or not math.isfinite(depth_scale):
            raise ValueError("depth_scale must be finite and greater than zero")
        color_image, depth_image = self.get_camera_images(wait_timeout=wait_timeout)
        if color_image is None or depth_image is None:
            raise RobotAPIError("no synchronized camera images are available")

        try:
            import cv2
            import numpy as np
        except ImportError as exception:
            raise RobotAPIError("saving camera images requires OpenCV and NumPy") from exception

        color_path = Path(color_image_path).expanduser()
        depth_path = Path(depth_image_path).expanduser()
        if create_directories:
            color_path.parent.mkdir(parents=True, exist_ok=True)
            depth_path.parent.mkdir(parents=True, exist_ok=True)

        depth_to_save = depth_image
        if np.issubdtype(depth_image.dtype, np.floating):
            scaled = np.nan_to_num(
                depth_image * depth_scale,
                nan=0.0,
                posinf=float(np.iinfo(np.uint16).max),
                neginf=0.0,
            )
            depth_to_save = np.clip(
                scaled, 0, np.iinfo(np.uint16).max
            ).astype(np.uint16)

        if not cv2.imwrite(str(color_path), color_image):
            raise RobotAPIError(f"failed to save color image to '{color_path}'")
        if not cv2.imwrite(str(depth_path), depth_to_save):
            raise RobotAPIError(f"failed to save depth image to '{depth_path}'")
        return color_path, depth_path

    def save_images(
        self,
        color_image_path: str,
        depth_image_path: str,
        **kwargs,
    ) -> Tuple[Path, Path]:
        """Alias for :meth:`save_camera_images`."""
        return self.save_camera_images(color_image_path, depth_image_path, **kwargs)

    def wait_until_ready(self, timeout: Optional[float] = None) -> None:
        """Wait until all robot services are available."""
        wait_timeout = self._service_wait_timeout if timeout is None else float(timeout)
        if wait_timeout <= 0.0:
            raise ValueError("timeout must be greater than zero")
        for client in (
            self._cartesian_client,
            self._joint_client,
            self._group_state_client,
            self._move_l_client,
            self._gripper_client,
            self._state_client,
        ):
            if not client.wait_for_service(timeout_sec=wait_timeout):
                raise ServiceUnavailableError(f"service '{client.srv_name}' is unavailable")

    def wait_for_sim_time(self, timeout: float = 5.0) -> float:
        """Wait for a nonzero, advancing ROS simulation clock and return seconds."""
        if timeout <= 0.0:
            raise ValueError("timeout must be greater than zero")
        if not self._use_sim_time:
            raise RobotAPIError("this RobotAPI node does not use simulated time")

        # Use a wall-clock deadline because ROS time cannot time out while paused.
        deadline = time.monotonic() + float(timeout)
        previous_nanoseconds = self._node.get_clock().now().nanoseconds
        while time.monotonic() < deadline:
            current_nanoseconds = self._node.get_clock().now().nanoseconds
            if (
                current_nanoseconds > 0
                and previous_nanoseconds > 0
                and current_nanoseconds != previous_nanoseconds
            ):
                return current_nanoseconds / 1e9
            previous_nanoseconds = current_nanoseconds
            time.sleep(0.02)

        raise ServiceCallTimeoutError(
            f"ROS simulation clock did not start advancing within {timeout:.1f} seconds"
        )

    def close(self) -> None:
        """Stop the executor and release this API's ROS resources."""
        with self._call_lock:
            if self._closed:
                return
            self._closed = True

            # Avoid retaining this instance until interpreter shutdown after an
            # application has already closed it explicitly.
            atexit.unregister(self._atexit_callback)
            self._executor.shutdown(timeout_sec=2.0)
            self._executor.remove_node(self._node)
            self._node.destroy_node()
            if self._context.ok():
                self._context.shutdown()
        self._spin_thread.join(timeout=2.0)

    def __enter__(self) -> "RobotAPI":
        try:
            self.wait_until_ready()
            return self
        except Exception:
            self.close()
            raise

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        del exc_type, exc_value, traceback
        self.close()

    def _call(self, client, request, timeout: Optional[float]) -> ServiceResult:
        """Call one service while the background executor handles its response."""
        response = self._call_response(client, request, timeout)
        return ServiceResult(success=bool(response.success), message=str(response.message))

    def _call_response(self, client, request, timeout: Optional[float]):
        """Return a raw service response after applying common timeout/error handling."""
        call_timeout = self._call_timeout if timeout is None else float(timeout)
        if call_timeout <= 0.0:
            raise ValueError("timeout must be greater than zero")

        with self._call_lock:
            if self._closed:
                raise RobotAPIError("RobotAPI is closed")
            if not client.wait_for_service(timeout_sec=self._service_wait_timeout):
                raise ServiceUnavailableError(f"service '{client.srv_name}' is unavailable")

            future = client.call_async(request)
            completed = threading.Event()
            future.add_done_callback(lambda _: completed.set())
            if not completed.wait(call_timeout):
                future.cancel()
                raise ServiceCallTimeoutError(
                    f"service '{client.srv_name}' did not respond within "
                    f"{call_timeout:.1f} seconds"
                )

            exception = future.exception()
            if exception is not None:
                raise RobotAPIError(
                    f"service '{client.srv_name}' failed: {exception}"
                ) from exception
            response = future.result()
            if response is None:
                raise RobotAPIError(f"service '{client.srv_name}' returned no response")

            if not bool(response.success) and self._raise_on_failure:
                raise RobotAPIError(
                    str(response.message)
                    or f"service '{client.srv_name}' reported failure"
                )
            return response

    def _decode_compressed_rgb(self, rgb_message):
        """Decode JPEG/PNG through CvBridge and stateful H.264 through PyAV."""
        image_format = str(rgb_message.format).lower()
        if "h264" not in image_format:
            return self._cv_bridge.compressed_imgmsg_to_cv2(
                rgb_message, desired_encoding=self._rgb_encoding
            )

        # H.264 messages form a stream, so retain decoder state between callbacks.
        if self._h264_decoder is None:
            try:
                import av
            except ImportError as exception:
                raise RobotAPIError(
                    "decoding H.264 camera images requires the PyAV package"
                ) from exception
            self._h264_decoder = av.CodecContext.create("h264", "r")

        frames = []
        for packet in self._h264_decoder.parse(bytes(rgb_message.data)):
            frames.extend(self._h264_decoder.decode(packet))
        if not frames:
            return None

        pixel_formats = {
            "bgr8": "bgr24",
            "rgb8": "rgb24",
            "bgra8": "bgra",
            "rgba8": "rgba",
            "mono8": "gray",
        }
        pixel_format = pixel_formats.get(self._rgb_encoding)
        if pixel_format is None:
            raise RobotAPIError(
                f"unsupported H.264 RGB encoding '{self._rgb_encoding}'"
            )
        return frames[-1].to_ndarray(format=pixel_format)

    def _camera_callback(self, rgb_message, depth_message) -> None:
        """Convert and retain one synchronized ROS image pair."""
        try:
            # Decode RGB according to the transport selected during subscription.
            if self._rgb_compressed:
                color = self._decode_compressed_rgb(rgb_message)
                if color is None:
                    return
            else:
                color = self._cv_bridge.imgmsg_to_cv2(
                    rgb_message, desired_encoding=self._rgb_encoding
                )
            depth = self._cv_bridge.imgmsg_to_cv2(
                depth_message, desired_encoding=self._depth_encoding
            )
            if depth is None:
                return
        except Exception as exception:  # CvBridgeError differs between ROS releases.
            self._node.get_logger().error(f"camera image conversion failed: {exception}")
            return

        with self._camera_lock:
            self._rgb_image = color.copy()
            self._depth_image = depth.copy()
            self._camera_event.set()

    def _transform_from_matrix(
        self,
        parent_frame: str,
        child_frame: str,
        transform_matrix: Sequence[Sequence[float]],
    ) -> TransformStamped:
        """Convert and validate a homogeneous matrix as a stamped TF message."""
        if not parent_frame or not child_frame:
            raise ValueError("parent_frame and child_frame must not be empty")
        if parent_frame == child_frame:
            raise ValueError("parent_frame and child_frame must be different")

        try:
            import numpy as np
        except ImportError as exception:
            raise RobotAPIError("matrix transforms require NumPy") from exception

        matrix = np.asarray(transform_matrix, dtype=np.float64)
        if matrix.shape != (4, 4):
            raise ValueError("transform_matrix must have shape (4, 4)")
        if not np.all(np.isfinite(matrix)):
            raise ValueError("transform_matrix values must all be finite")
        if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-6):
            raise ValueError("transform_matrix must be a homogeneous 4-by-4 matrix")

        rotation = matrix[:3, :3]
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5):
            raise ValueError("transform_matrix rotation must be orthonormal")
        if not np.isclose(np.linalg.det(rotation), 1.0, atol=1e-5):
            raise ValueError("transform_matrix rotation determinant must be +1")

        quaternion = self._quaternion_from_rotation_matrix(rotation)
        transform = TransformStamped()
        transform.header.stamp = self._node.get_clock().now().to_msg()
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(matrix[0, 3])
        transform.transform.translation.y = float(matrix[1, 3])
        transform.transform.translation.z = float(matrix[2, 3])
        transform.transform.rotation.x = quaternion[0]
        transform.transform.rotation.y = quaternion[1]
        transform.transform.rotation.z = quaternion[2]
        transform.transform.rotation.w = quaternion[3]
        return transform

    @staticmethod
    def _quaternion_from_rotation_matrix(rotation) -> Tuple[float, float, float, float]:
        """Convert a 3-by-3 NumPy rotation matrix to an XYZW quaternion."""
        trace = float(rotation[0, 0] + rotation[1, 1] + rotation[2, 2])
        if trace > 0.0:
            scale = math.sqrt(trace + 1.0) * 2.0
            x = (rotation[2, 1] - rotation[1, 2]) / scale
            y = (rotation[0, 2] - rotation[2, 0]) / scale
            z = (rotation[1, 0] - rotation[0, 1]) / scale
            w = 0.25 * scale
        elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
            scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            x = 0.25 * scale
            y = (rotation[0, 1] + rotation[1, 0]) / scale
            z = (rotation[0, 2] + rotation[2, 0]) / scale
            w = (rotation[2, 1] - rotation[1, 2]) / scale
        elif rotation[1, 1] > rotation[2, 2]:
            scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            x = (rotation[0, 1] + rotation[1, 0]) / scale
            y = 0.25 * scale
            z = (rotation[1, 2] + rotation[2, 1]) / scale
            w = (rotation[0, 2] - rotation[2, 0]) / scale
        else:
            scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            x = (rotation[0, 2] + rotation[2, 0]) / scale
            y = (rotation[1, 2] + rotation[2, 1]) / scale
            z = 0.25 * scale
            w = (rotation[1, 0] - rotation[0, 1]) / scale

        norm = math.sqrt(x * x + y * y + z * z + w * w)
        return x / norm, y / norm, z / norm, w / norm

    @staticmethod
    def _parse_state_message(message: str) -> Tuple[str, Dict[str, float]]:
        """Parse ``group=<name>; joints={name: value, ...}`` from Trigger."""
        try:
            group_part, joints_part = message.split(";", maxsplit=1)
            group = group_part.split("=", maxsplit=1)[1].strip()
            body = joints_part.split("=", maxsplit=1)[1].strip().strip("{}")
            joints: Dict[str, float] = {}
            if body:
                for entry in body.split(","):
                    name, value = entry.split(":", maxsplit=1)
                    joints[name.strip()] = float(value)
            return group, joints
        except (IndexError, ValueError) as exception:
            raise RobotAPIError(f"unable to parse state response: {message}") from exception


__all__ = [
    "RobotAPI",
    "RobotAPIError",
    "RobotStateResult",
    "MoveLResult",
    "ServiceCallTimeoutError",
    "ServiceResult",
    "ServiceUnavailableError",
]
