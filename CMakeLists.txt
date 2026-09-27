cmake_minimum_required(VERSION 3.8)
project(prismatic_fr3_duo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# ------------------------------------------------------------------------------
# Define .srv interfaces
# ------------------------------------------------------------------------------
set(SRV_FILES
  "srv/Move3DPose.srv"
  "srv/MoveJointPose.srv"
  "srv/MoveGroupState.srv"
  "srv/MoveL.srv"
  "srv/GripperCommand.srv"
)

set(SRV_DEPENDENCIES
  geometry_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${SRV_FILES}
  DEPENDENCIES ${SRV_DEPENDENCIES}
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)

# Install the implementation and its stable public import facade.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(FILES
    scripts/robot_api.py
    scripts/robot_motion_api.py
  DESTINATION ${PYTHON_INSTALL_DIR})

# ------------------------------------------------------------------------------
# Build C++ executable (server node)
# ------------------------------------------------------------------------------

# Robot motion
add_executable(robot_motion_server src/robot_motion_server.cpp)
ament_target_dependencies(robot_motion_server
  rclcpp
  rcl_interfaces
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  sensor_msgs
  std_srvs
  trajectory_msgs
  visualization_msgs
)
target_link_libraries(robot_motion_server "${cpp_typesupport_target}")

# Gripper command
add_executable(gripper_server src/gripper_server.cpp)
ament_target_dependencies(gripper_server
  rclcpp
  rcl_interfaces
  moveit_ros_planning_interface
)
target_include_directories(gripper_server PRIVATE include)
target_include_directories(robot_motion_server PRIVATE include)
target_link_libraries(gripper_server "${cpp_typesupport_target}")

# Install file
install(TARGETS
    gripper_server
    robot_motion_server
  DESTINATION lib/${PROJECT_NAME})

# Install the RViz target helper as a ROS package executable.
install(PROGRAMS
    scripts/publish_ik_targets.sh
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

# install(FILES README.md
#   DESTINATION share/${PROJECT_NAME})

# install(FILES scripts/README.md
#   DESTINATION share/${PROJECT_NAME}/scripts)

# Build test files
if(BUILD_TESTING)
find_package(ament_lint_auto REQUIRED)
# the following line skips the linter which checks for copyrights
# comment the line when a copyright and license is added to all source files
set(ament_cmake_copyright_FOUND TRUE)
# the following line skips cpplint (only works in a git repo)
# comment the line when this package is in a git repo and when
# a copyright and license is added to all source files
set(ament_cmake_cpplint_FOUND TRUE)
ament_lint_auto_find_test_dependencies()
endif()

# ------------------------------------------------------------------------------
# Export runtime dependencies
# ------------------------------------------------------------------------------
ament_export_dependencies(rosidl_default_runtime)
ament_package()
