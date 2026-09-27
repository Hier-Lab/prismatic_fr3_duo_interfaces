#!/usr/bin/env bash

# Publish left and right Cartesian targets as TF frames for RViz inspection.
# Each pose must use the order: x y z qx qy qz qw.

set -u

show_usage() {
  echo "Usage:"
  echo "  $0 \"LEFT_POSE\" \"RIGHT_POSE\" [PARENT_FRAME]"
  echo
  echo "Example:"
  echo "  $0 \"0.66 0.20 0.18 0 0 0 1\" \"0.66 -0.20 0.18 0 0 0 1\""
  echo
  echo "Set USE_SIM_TIME=true when visualizing against a simulated robot."
}

if [[ ${1:-} == "-h" || ${1:-} == "--help" ]]; then
  show_usage
  exit 0
fi

# Prompt interactively when no pose arguments were provided.
if [[ $# -eq 0 ]]; then
  read -r -p "Left pose  [x y z qx qy qz qw]: " left_pose_text
  read -r -p "Right pose [x y z qx qy qz qw]: " right_pose_text
  parent_frame="world"
elif [[ $# -eq 2 || $# -eq 3 ]]; then
  left_pose_text=$1
  right_pose_text=$2
  parent_frame=${3:-world}
else
  show_usage >&2
  exit 2
fi

read -r -a left_pose <<< "${left_pose_text}"
read -r -a right_pose <<< "${right_pose_text}"

validate_pose() {
  local label=$1
  shift
  local values=("$@")
  local number_pattern='^-?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$'

  if [[ ${#values[@]} -ne 7 ]]; then
    echo "${label} pose must contain exactly 7 values: x y z qx qy qz qw" >&2
    return 1
  fi

  # Reject malformed values before starting either ROS process.
  for value in "${values[@]}"; do
    if [[ ! $value =~ $number_pattern ]]; then
      echo "${label} pose contains a non-numeric value: ${value}" >&2
      return 1
    fi
  done
}

validate_pose "Left" "${left_pose[@]}" || exit 2
validate_pose "Right" "${right_pose[@]}" || exit 2

if [[ ! $parent_frame =~ ^[A-Za-z][A-Za-z0-9_/]*$ ]]; then
  echo "Parent frame is not a valid TF frame name: ${parent_frame}" >&2
  exit 2
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ros2 was not found. Source ROS 2 before running this script." >&2
  exit 1
fi

clock_arguments=()
if [[ ${USE_SIM_TIME:-false} == "true" ]]; then
  clock_arguments=(--ros-args -p use_sim_time:=true)
fi

start_publisher() {
  local child_frame=$1
  shift
  local pose=("$@")

  ros2 run tf2_ros static_transform_publisher \
    --x "${pose[0]}" --y "${pose[1]}" --z "${pose[2]}" \
    --qx "${pose[3]}" --qy "${pose[4]}" --qz "${pose[5]}" --qw "${pose[6]}" \
    --frame-id "${parent_frame}" --child-frame-id "${child_frame}" \
    "${clock_arguments[@]}" &
  publisher_pid=$!
}

# Start separate publishers because static_transform_publisher publishes one TF.
start_publisher "left_ik_target" "${left_pose[@]}"
left_pid=$publisher_pid
start_publisher "right_ik_target" "${right_pose[@]}"
right_pid=$publisher_pid

cleanup() {
  # Stop both background publishers when the user presses Ctrl+C.
  trap - EXIT INT TERM
  kill "${left_pid}" "${right_pid}" 2>/dev/null || true
  wait "${left_pid}" "${right_pid}" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

echo "Publishing targets relative to '${parent_frame}':"
echo "  left_ik_target:  ${left_pose[*]}"
echo "  right_ik_target: ${right_pose[*]}"

# Give the static transforms time to become visible before requesting IK.
echo "Waiting 2 seconds before calling /compute_ik..."
sleep 2

# Construct one multi-tip IK request directly from the two command-line poses.
ik_request="{
  ik_request: {
    group_name: 'dual_arm',
    robot_state: {is_diff: true},
    avoid_collisions: true,
    ik_link_names: ['left_fr3_hand_tcp', 'right_fr3_hand_tcp'],
    pose_stamped_vector: [
      {
        header: {frame_id: '${parent_frame}'},
        pose: {
          position: {x: ${left_pose[0]}, y: ${left_pose[1]}, z: ${left_pose[2]}},
          orientation: {
            x: ${left_pose[3]}, y: ${left_pose[4]},
            z: ${left_pose[5]}, w: ${left_pose[6]}
          }
        }
      },
      {
        header: {frame_id: '${parent_frame}'},
        pose: {
          position: {x: ${right_pose[0]}, y: ${right_pose[1]}, z: ${right_pose[2]}},
          orientation: {
            x: ${right_pose[3]}, y: ${right_pose[4]},
            z: ${right_pose[5]}, w: ${right_pose[6]}
          }
        }
      }
    ],
    timeout: {sec: 2, nanosec: 0}
  }
}"

if ! ros2 service call \
  /compute_ik moveit_msgs/srv/GetPositionIK "${ik_request}"
then
  echo "The /compute_ik service call failed; TF targets remain available." >&2
fi

echo "Open RViz, add the TF display, and press Ctrl+C here when finished."

# Keep the script alive while either publisher is running.
wait "${left_pid}" "${right_pid}"
