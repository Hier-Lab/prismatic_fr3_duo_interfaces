#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Geometry>  // NOLINT(build/include_order)
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/srv/get_position_ik.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "prismatic_fr3_duo_interfaces/srv/move3_d_pose.hpp"
#include "prismatic_fr3_duo_interfaces/srv/move_group_state.hpp"
#include "prismatic_fr3_duo_interfaces/srv/move_joint_pose.hpp"
#include "prismatic_fr3_duo_interfaces/srv/move_l.hpp"
#include "prismatic_fr3_duo_interfaces/moveit_model_parameters.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using Move3DPose = prismatic_fr3_duo_interfaces::srv::Move3DPose;
using MoveGroupState = prismatic_fr3_duo_interfaces::srv::MoveGroupState;
using MoveJointPose = prismatic_fr3_duo_interfaces::srv::MoveJointPose;
using MoveL = prismatic_fr3_duo_interfaces::srv::MoveL;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;

namespace
{
// Read a parameter that may already have been declared automatically from launch overrides.
// This avoids redeclaration errors while still providing a standalone default value.
template<typename T>
T declare_or_get(
  const rclcpp::Node::SharedPtr & node, const std::string & name,
  const T & default_value)
{
  if (!node->has_parameter(name)) {
    return node->declare_parameter<T>(name, default_value);
  }
  return node->get_parameter(name).get_value<T>();
}

bool is_finite(double value)
{
  return std::isfinite(value);
}

bool finite_pose(const geometry_msgs::msg::Pose & pose)
{
  return is_finite(pose.position.x) && is_finite(pose.position.y) &&
         is_finite(pose.position.z) && is_finite(pose.orientation.x) &&
         is_finite(pose.orientation.y) && is_finite(pose.orientation.z) &&
         is_finite(pose.orientation.w);
}

double quaternion_norm(const geometry_msgs::msg::Quaternion & quaternion)
{
  return std::sqrt(
    quaternion.x * quaternion.x + quaternion.y * quaternion.y +
    quaternion.z * quaternion.z + quaternion.w * quaternion.w);
}

void normalize(geometry_msgs::msg::Quaternion & quaternion)
{
  const double norm = quaternion_norm(quaternion);
  quaternion.x /= norm;
  quaternion.y /= norm;
  quaternion.z /= norm;
  quaternion.w /= norm;
}

geometry_msgs::msg::Quaternion multiply(
  const geometry_msgs::msg::Quaternion & lhs,
  const geometry_msgs::msg::Quaternion & rhs)
{
  geometry_msgs::msg::Quaternion result;
  result.w = lhs.w * rhs.w - lhs.x * rhs.x - lhs.y * rhs.y - lhs.z * rhs.z;
  result.x = lhs.w * rhs.x + lhs.x * rhs.w + lhs.y * rhs.z - lhs.z * rhs.y;
  result.y = lhs.w * rhs.y - lhs.x * rhs.z + lhs.y * rhs.w + lhs.z * rhs.x;
  result.z = lhs.w * rhs.z + lhs.x * rhs.y - lhs.y * rhs.x + lhs.z * rhs.w;
  normalize(result);
  return result;
}

bool has_trajectory(const MoveGroup::Plan & plan)
{
  // A successful MoveIt result is not sufficient: reject plans with no executable points.
  return !plan.trajectory_.joint_trajectory.points.empty() ||
         !plan.trajectory_.multi_dof_joint_trajectory.points.empty();
}

}  // namespace

class RobotMotionServer
{
public:
  RobotMotionServer(const std::string & node_name, const rclcpp::NodeOptions & options)
  {
    // Keep MoveIt communication and service callbacks on separate nodes. The multi-threaded
    // executor can then process MoveIt action/service responses while a user service waits.
    moveit_node_ = std::make_shared<rclcpp::Node>("robot_motion_moveit", options);
    // Reuse launch-supplied options so both nodes select the same system or simulation clock.
    service_node_ = std::make_shared<rclcpp::Node>(node_name, options);

    // Attach to the model already loaded by move_group instead of importing a robot-specific
    // MoveIt configuration package into this generic interface process.
    move_group_node_ = declare_or_get<std::string>(
      moveit_node_, "move_group_node", "/move_group");
    move_group_namespace_ = declare_or_get<std::string>(
      moveit_node_, "move_group_namespace", "");
    const double model_parameter_timeout = declare_or_get<double>(
      moveit_node_, "move_group_parameter_timeout", 10.0);
    prismatic_fr3_duo_interfaces::ensure_moveit_model_parameters(
      moveit_node_, move_group_node_, model_parameter_timeout);

    // An empty request uses default_group_; an optional allowlist can restrict explicit groups.
    default_group_ = declare_or_get<std::string>(
      moveit_node_, "default_planning_group",
      "whole_body");
    const auto groups = declare_or_get<std::vector<std::string>>(
      moveit_node_, "supported_planning_groups",
      {});
    supported_groups_.insert(groups.begin(), groups.end());
    if (!supported_groups_.empty() && supported_groups_.count(default_group_) == 0U) {
      throw std::runtime_error("default_planning_group must be in supported_planning_groups");
    }

    planning_time_ = declare_or_get<double>(moveit_node_, "planning_time", 5.0);
    planning_attempts_ = declare_or_get<int>(moveit_node_, "planning_attempts", 10);
    velocity_scale_ = declare_or_get<double>(moveit_node_, "velocity_scale", 0.5);
    acceleration_scale_ = declare_or_get<double>(moveit_node_, "acceleration_scale", 0.5);
    ik_timeout_ = declare_or_get<double>(moveit_node_, "ik_timeout", 0.25);
    if (ik_timeout_ <= 0.0) {
      throw std::runtime_error("ik_timeout must be greater than zero");
    }
    compute_ik_service_ = declare_or_get<std::string>(
      moveit_node_, "compute_ik_service", "/compute_ik");
    display_trajectory_topic_ = declare_or_get<std::string>(
      moveit_node_, "display_trajectory_topic", "/display_planned_path");
    if (display_trajectory_topic_.empty()) {
      throw std::runtime_error("display_trajectory_topic must not be empty");
    }

    // MoveL converts the global joint trajectory back to task-space references
    // consumed by the operational-space ros2_control controller.
    tracking_tip_links_ = declare_or_get<std::vector<std::string>>(
      moveit_node_, "tracking_tip_links",
      {"left_fr3_hand_tcp", "right_fr3_hand_tcp"});
    display_ee_waypoint_topics_ = declare_or_get<std::vector<std::string>>(
      moveit_node_, "display_ee_waypoint_topics",
      {"/display_planned_waypoints/left_ee", "/display_planned_waypoints/right_ee"});
    display_ee_waypoint_size_ = declare_or_get<double>(
      moveit_node_, "display_ee_waypoint_size", 0.02);
    tracking_pose_topics_ = declare_or_get<std::vector<std::string>>(
      moveit_node_, "tracking_pose_topics",
      {"/whole_body_controller/target_pose/left", "/whole_body_controller/target_pose/right"});
    tracking_velocity_topics_ = declare_or_get<std::vector<std::string>>(
      moveit_node_, "tracking_velocity_topics",
      {"/whole_body_controller/target_velocity/left",
        "/whole_body_controller/target_velocity/right"});
    current_pose_topics_ = declare_or_get<std::vector<std::string>>(
      moveit_node_, "current_pose_topics",
      {"/whole_body_controller/current_pose/left", "/whole_body_controller/current_pose/right"});
    current_velocity_topics_ = declare_or_get<std::vector<std::string>>(
      moveit_node_, "current_velocity_topics",
      {"/whole_body_controller/current_velocity/left",
        "/whole_body_controller/current_velocity/right"});
    joint_state_topic_ = declare_or_get<std::string>(
      moveit_node_, "joint_state_topic", "/joint_states");
    tracking_joint_topic_ = declare_or_get<std::string>(
      moveit_node_, "tracking_joint_topic", "/whole_body_controller/target_joint");
    tracking_frame_ = declare_or_get<std::string>(
      moveit_node_, "tracking_frame", "rail_link");
    tracking_publish_rate_ = declare_or_get<double>(
      moveit_node_, "tracking_publish_rate", 200.0);
    interpolate_velocity_ = declare_or_get<bool>(
      moveit_node_, "interpolate_velocity", true);
    default_position_tolerance_ = declare_or_get<double>(
      moveit_node_, "tracking_position_tolerance", 0.01);
    default_orientation_tolerance_ = declare_or_get<double>(
      moveit_node_, "tracking_orientation_tolerance", 0.05);
    default_tracking_timeout_ = declare_or_get<double>(
      moveit_node_, "tracking_timeout", 5.0);
    require_tracking_subscribers_ = declare_or_get<bool>(
      moveit_node_, "require_tracking_subscribers", true);
    if (tracking_tip_links_.empty() || tracking_frame_.empty() ||
      tracking_tip_links_.size() != tracking_pose_topics_.size() ||
      tracking_tip_links_.size() != tracking_velocity_topics_.size() ||
      tracking_tip_links_.size() != current_pose_topics_.size() ||
      tracking_tip_links_.size() != current_velocity_topics_.size() ||
      tracking_tip_links_.size() != display_ee_waypoint_topics_.size())
    {
      throw std::runtime_error(
              "tracking tip, target pose/velocity, current pose/velocity, and display marker "
              "topic arrays must be equal length");
    }
    if (display_ee_waypoint_size_ <= 0.0 || !is_finite(tracking_publish_rate_) ||
      tracking_publish_rate_ <= 0.0 ||
      default_position_tolerance_ <= 0.0 ||
      default_orientation_tolerance_ <= 0.0 || default_tracking_timeout_ <= 0.0)
    {
      throw std::runtime_error("MoveL rate, tolerances, and timeout must be greater than zero");
    }

    // IK is delegated to move_group because it owns the active kinematics configuration and
    // planning scene. This is especially important for dual-arm, multi-tip IK requests.
    compute_ik_client_ = moveit_node_->create_client<moveit_msgs::srv::GetPositionIK>(
      compute_ik_service_);

    // Latch the latest plan so RViz can display it even when RViz connects after planning.
    display_trajectory_publisher_ = service_node_->create_publisher<
      moveit_msgs::msg::DisplayTrajectory>(
      display_trajectory_topic_, rclcpp::QoS(1).transient_local());
    for (const auto & topic : display_ee_waypoint_topics_) {
      display_ee_waypoint_publishers_.push_back(
        service_node_->create_publisher<visualization_msgs::msg::Marker>(
          topic, rclcpp::QoS(1).transient_local()));
    }

    // Use depth-one command topics because only the latest tracking sample matters.
    for (const auto & topic : tracking_pose_topics_) {
      tracking_pose_publishers_.push_back(
        service_node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic, rclcpp::QoS(1)));
    }
    for (const auto & topic : tracking_velocity_topics_) {
      tracking_velocity_publishers_.push_back(
        service_node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic, rclcpp::QoS(1)));
    }
    for (const auto & topic : current_pose_topics_) {
      current_pose_publishers_.push_back(
        service_node_->create_publisher<geometry_msgs::msg::PoseStamped>(topic, rclcpp::QoS(1)));
    }
    for (const auto & topic : current_velocity_topics_) {
      current_velocity_publishers_.push_back(
        service_node_->create_publisher<geometry_msgs::msg::TwistStamped>(topic, rclcpp::QoS(1)));
    }
    tracking_joint_publisher_ = service_node_->create_publisher<sensor_msgs::msg::JointState>(
      tracking_joint_topic_, rclcpp::QoS(1));

    // Cache measured q_dot directly from joint_states. MoveGroupInterface's current RobotState
    // reliably carries q, but its state-monitor copy does not always retain velocity metadata.
    joint_state_subscription_ = service_node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr message) {
        if (message->name.size() != message->velocity.size()) {
          RCLCPP_WARN_THROTTLE(
            service_node_->get_logger(), *service_node_->get_clock(), 1000,
            "Joint state velocity array is missing or has the wrong size on '%s'",
            joint_state_topic_.c_str());
          return;
        }
        std::lock_guard<std::mutex> lock(joint_velocity_mutex_);
        measured_joint_velocities_.clear();
        for (std::size_t index = 0; index < message->name.size(); ++index) {
          measured_joint_velocities_[message->name[index]] = message->velocity[index];
        }
      });

    // Public service API: Cartesian, joint, predefined SRDF state, and state reporting.
    pose_service_ = service_node_->create_service<Move3DPose>(
      "move_cartesian", std::bind(&RobotMotionServer::handle_pose_request, this, _1, _2));
    joint_service_ = service_node_->create_service<MoveJointPose>(
      "move_joint", std::bind(&RobotMotionServer::handle_joint_request, this, _1, _2));
    group_state_service_ = service_node_->create_service<MoveGroupState>(
      "move_group_state",
      std::bind(&RobotMotionServer::handle_group_state_request, this, _1, _2));
    move_l_service_ = service_node_->create_service<MoveL>(
      "move_l", std::bind(&RobotMotionServer::handle_move_l_request, this, _1, _2));
    state_service_ = service_node_->create_service<std_srvs::srv::Trigger>(
      "get_current_state", std::bind(&RobotMotionServer::handle_state_request, this, _1, _2));

    RCLCPP_INFO(
      service_node_->get_logger(),
      "Robot motion services ready (default group: %s)", default_group_.c_str());
  }

  rclcpp::Node::SharedPtr get_service_node() const {return service_node_;}
  rclcpp::Node::SharedPtr get_moveit_node() const {return moveit_node_;}

private:
  // Return a cached MoveGroupInterface for the requested SRDF group. Interfaces are created
  // lazily so the server makes no assumptions about a robot's group names.
  std::shared_ptr<MoveGroup> get_move_group(
    const std::string & requested_group,
    std::string & error)
  {
    const std::string group = requested_group.empty() ? default_group_ : requested_group;
    if (!supported_groups_.empty() && supported_groups_.count(group) == 0U) {
      std::ostringstream stream;
      stream << "Unsupported planning group '" << group << "'. Allowed groups: ";
      bool first = true;
      for (const auto & supported : supported_groups_) {
        stream << (first ? "" : ", ") << supported;
        first = false;
      }
      error = stream.str();
      return nullptr;
    }

    const auto existing = move_groups_.find(group);
    if (existing != move_groups_.end()) {
      return existing->second;
    }

    try {
      // Every group uses the same planning limits so behavior is consistent across requests.
      const MoveGroup::Options options(group, "robot_description", move_group_namespace_);
      auto move_group = std::make_shared<MoveGroup>(moveit_node_, options);
      move_group->setPlanningTime(planning_time_);
      move_group->setNumPlanningAttempts(planning_attempts_);
      move_group->setMaxVelocityScalingFactor(velocity_scale_);
      move_group->setMaxAccelerationScalingFactor(acceleration_scale_);
      move_group->startStateMonitor();
      move_groups_.emplace(group, move_group);
      RCLCPP_INFO(
        service_node_->get_logger(), "Initialized MoveIt group '%s' (planning frame: %s)",
        group.c_str(), move_group->getPlanningFrame().c_str());
      return move_group;
    } catch (const std::exception & exception) {
      error = "Unable to initialize planning group '" + group + "': " + exception.what();
      return nullptr;
    }
  }

  bool resolve_end_effector(
    const moveit::core::RobotModelConstPtr & model,
    const moveit::core::JointModelGroup * planning_group,
    const std::string & requested_name,
    std::string & tip_link,
    std::string & error) const
  {
    if (requested_name.empty()) {
      error = "End-effector names must not be empty";
      return false;
    }

    // Accept either an SRDF semantic name such as "left_ee" or a concrete tip link such as
    // "left_fr3_hand_tcp". MoveIt IK ultimately requires the concrete link name.
    if (model->hasEndEffector(requested_name)) {
      const auto * end_effector = model->getEndEffector(requested_name);
      tip_link = end_effector->getEndEffectorParentGroup().second;
    } else if (model->hasLinkModel(requested_name)) {
      tip_link = requested_name;
    } else {
      error = "Unknown end effector or tip link '" + requested_name + "'";
      return false;
    }

    // Reject a valid robot link when it is outside the selected planning group. For example,
    // right_ee cannot be controlled through the left_arm group.
    if (tip_link.empty() || !planning_group->hasLinkModel(tip_link)) {
      error = "End effector '" + requested_name + "' is not part of planning group '" +
        planning_group->getName() + "'";
      return false;
    }
    return true;
  }

  template<typename ResponseT>
  void set_response(
    const std::shared_ptr<ResponseT> & response,
    bool success, const std::string & message) const
  {
    response->success = success;
    response->message = message;
    if (success) {
      RCLCPP_INFO(service_node_->get_logger(), "%s", message.c_str());
    } else {
      RCLCPP_WARN(service_node_->get_logger(), "%s", message.c_str());
    }
  }

  void publish_display_trajectory(
    const std::shared_ptr<MoveGroup> & move_group, const MoveGroup::Plan & plan) const
  {
    // DisplayTrajectory carries the measured planning start plus the complete timed robot path.
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = move_group->getRobotModel()->getName();
    display.trajectory_start = plan.start_state_;
    display.trajectory.push_back(plan.trajectory_);
    display_trajectory_publisher_->publish(display);
    publish_end_effector_waypoints(move_group, plan);
  }

  void publish_end_effector_waypoints(
    const std::shared_ptr<MoveGroup> & move_group, const MoveGroup::Plan & plan) const
  {
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if (trajectory.joint_names.empty() || trajectory.points.empty()) {
      RCLCPP_WARN(
        service_node_->get_logger(), "Cannot display TCP waypoints for an empty joint path");
      return;
    }

    // Reconstruct the exact planning start state before applying sampled trajectory positions.
    moveit::core::RobotState start_state(move_group->getRobotModel());
    start_state.setToDefaultValues();
    if (!moveit::core::robotStateMsgToRobotState(plan.start_state_, start_state, true)) {
      RCLCPP_WARN(
        service_node_->get_logger(), "Cannot decode the plan start state for TCP waypoints");
      return;
    }
    start_state.updateLinkTransforms();

    const auto model = move_group->getRobotModel();
    for (const auto & tip : tracking_tip_links_) {
      if (!model->hasLinkModel(tip)) {
        RCLCPP_WARN(
          service_node_->get_logger(), "Cannot display unknown TCP link '%s'", tip.c_str());
        return;
      }
    }

    std::vector<visualization_msgs::msg::Marker> markers(tracking_tip_links_.size());
    for (std::size_t tip = 0; tip < markers.size(); ++tip) {
      auto & marker = markers[tip];
      marker.header.stamp = service_node_->now();
      marker.header.frame_id = model->getModelFrame();
      marker.ns = "planned_ee_waypoints";
      marker.id = static_cast<int>(tip);
      marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = display_ee_waypoint_size_;
      marker.scale.y = display_ee_waypoint_size_;
      marker.scale.z = display_ee_waypoint_size_;
      marker.color.a = 1.0F;
      marker.color.r = tip == 0U ? 0.1F : 1.0F;
      marker.color.g = tip == 0U ? 0.6F : 0.3F;
      marker.color.b = tip == 0U ? 1.0F : 0.1F;
      marker.points.reserve(trajectory.points.size());
    }

    // Run FK at every MoveIt joint waypoint and render only discrete spheres, without lines.
    for (const auto & point : trajectory.points) {
      if (point.positions.size() != trajectory.joint_names.size()) {
        RCLCPP_WARN(service_node_->get_logger(), "Cannot decode a joint waypoint for TCP markers");
        return;
      }

      moveit::core::RobotState sampled_state(start_state);
      for (std::size_t joint = 0; joint < trajectory.joint_names.size(); ++joint) {
        sampled_state.setVariablePosition(trajectory.joint_names[joint], point.positions[joint]);
      }
      for (std::size_t tip = 0; tip < tracking_tip_links_.size(); ++tip) {
        const auto pose = pose_from_state(sampled_state, tracking_tip_links_[tip]);
        markers[tip].points.push_back(pose.position);
      }
    }

    for (std::size_t tip = 0; tip < markers.size(); ++tip) {
      display_ee_waypoint_publishers_[tip]->publish(markers[tip]);
    }
  }

  template<typename ResponseT>
  bool plan_and_execute(
    const std::shared_ptr<MoveGroup> & move_group,
    const std::shared_ptr<ResponseT> & response,
    const std::string & description)
  {
    // Freeze one measured state into the request instead of relying on MoveIt's implicit scene
    // state, which may still contain an earlier SRDF/default posture.
    const auto start_state = move_group->getCurrentState(2.0);
    if (!start_state) {
      set_response(response, false, "Timed out waiting for the current robot state");
      return false;
    }
    start_state->updateLinkTransforms();
    move_group->setStartState(*start_state);

    // Planning and execution are shared by direct joint and named SRDF-state goals.
    MoveGroup::Plan plan;
    const auto plan_result = move_group->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      set_response(
        response, false, description + " planning failed: " +
        moveit::core::error_code_to_string(plan_result));
      return false;
    }
    if (!has_trajectory(plan)) {
      set_response(response, false, description + " planning produced an empty trajectory");
      return false;
    }

    publish_display_trajectory(move_group, plan);

    const auto execute_result = move_group->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      set_response(
        response, false, description + " execution failed: " +
        moveit::core::error_code_to_string(execute_result));
      return false;
    }

    set_response(response, true, description + " completed");
    return true;
  }

  template<typename ResponseT>
  bool plan_cartesian_request(
    const std::shared_ptr<Move3DPose::Request> request,
    const std::shared_ptr<ResponseT> & response,
    std::shared_ptr<MoveGroup> & move_group,
    MoveGroup::Plan & plan,
    moveit::core::RobotStatePtr & start_state)
  {
    // Validate the parallel request arrays before accessing any target by index.
    if (request->mode != Move3DPose::Request::ABSOLUTE &&
      request->mode != Move3DPose::Request::RELATIVE)
    {
      set_response(response, false, "Invalid pose mode; use ABSOLUTE (0) or RELATIVE (1)");
      return false;
    }
    if (request->mode == Move3DPose::Request::RELATIVE &&
      request->relative_frame != Move3DPose::Request::PLANNING_FRAME &&
      request->relative_frame != Move3DPose::Request::END_EFFECTOR_FRAME)
    {
      set_response(
        response, false,
        "Invalid relative_frame; use PLANNING_FRAME (0) or END_EFFECTOR_FRAME (1)");
      return false;
    }
    if (request->end_effector_names.empty()) {
      set_response(response, false, "At least one end effector is required");
      return false;
    }
    if (request->end_effector_names.size() != request->ee_poses.size()) {
      set_response(response, false, "end_effector_names and ee_poses must have equal length");
      return false;
    }

    std::string error;
    move_group = get_move_group(request->planning_group, error);
    if (!move_group) {
      set_response(response, false, error);
      return false;
    }
    const auto model = move_group->getRobotModel();
    const auto * joint_group = model->getJointModelGroup(move_group->getName());
    if (!joint_group) {
      set_response(response, false, "Robot model does not contain the selected planning group");
      return false;
    }

    // Capture one measured state and reuse it for relative goals, IK, planning, and streaming.
    start_state = move_group->getCurrentState(2.0);
    if (!start_state) {
      set_response(response, false, "Timed out waiting for the current robot state");
      return false;
    }
    start_state->updateLinkTransforms();
    move_group->clearPoseTargets();
    move_group->setStartState(*start_state);

    // Store targets by resolved tip link. The map makes it possible to fill omitted tips with
    // their current pose later when a multi-tip solver requires all arm poses.
    std::set<std::string> used_tip_links;
    std::map<std::string, geometry_msgs::msg::Pose> requested_targets;
    std::vector<std::string> requested_tips;

    for (std::size_t index = 0; index < request->end_effector_names.size(); ++index) {
      std::string tip_link;
      if (!resolve_end_effector(
          model, joint_group, request->end_effector_names[index], tip_link, error))
      {
        move_group->clearPoseTargets();
        set_response(response, false, error);
        return false;
      }
      if (!used_tip_links.insert(tip_link).second) {
        move_group->clearPoseTargets();
        set_response(response, false, "Each end effector may appear only once in a request");
        return false;
      }

      geometry_msgs::msg::Pose target = request->ee_poses[index];
      if (!finite_pose(target)) {
        move_group->clearPoseTargets();
        set_response(response, false, "Pose values must all be finite");
        return false;
      }

      if (request->mode == Move3DPose::Request::RELATIVE) {
        // Resolve the current pose once so translation and rotation use the same frame choice.
        const auto current = pose_from_state(*start_state, tip_link);
        Eigen::Vector3d offset(
          target.position.x, target.position.y, target.position.z);
        const Eigen::Quaterniond current_orientation(
          current.orientation.w, current.orientation.x,
          current.orientation.y, current.orientation.z);
        if (request->relative_frame == Move3DPose::Request::END_EFFECTOR_FRAME) {
          // Convert a tool-local XYZ offset into the fixed planning frame.
          offset = current_orientation * offset;
        }
        target.position.x = current.position.x + offset.x();
        target.position.y = current.position.y + offset.y();
        target.position.z = current.position.z + offset.z();

        const double delta_norm = quaternion_norm(target.orientation);
        if (delta_norm < 1e-9) {
          target.orientation = current.orientation;
        } else {
          // Rotation composition order selects fixed-frame versus tool-local axes.
          normalize(target.orientation);
          target.orientation = request->relative_frame == Move3DPose::Request::PLANNING_FRAME ?
            multiply(target.orientation, current.orientation) :
            multiply(current.orientation, target.orientation);
        }
      } else {
        // Absolute goals must contain a usable orientation. Normalize it to prevent numerical
        // errors caused by clients sending a slightly non-unit quaternion.
        const double norm = quaternion_norm(target.orientation);
        if (norm < 1e-9) {
          move_group->clearPoseTargets();
          set_response(response, false, "Absolute poses require a non-zero orientation quaternion");
          return false;
        }
        normalize(target.orientation);
      }

      requested_targets.emplace(tip_link, target);
      requested_tips.push_back(tip_link);
    }

    auto target_state = std::make_shared<moveit::core::RobotState>(*start_state);

    // Do not send pose constraints directly to OMPL. For composite groups that can produce an
    // empty sampleable goal region. Instead, compute one concrete IK state and plan to it.
    if (!compute_ik_client_->wait_for_service(std::chrono::seconds(2))) {
      set_response(
        response, false, "MoveIt IK service '" + compute_ik_service_ + "' is unavailable");
      return false;
    }

    std::vector<std::string> ik_tips;

    // Discover every end-effector tip contained in the selected group. SRDF parent-group
    // declarations are not always attached to dual_arm, so scan semantic end effectors too.
    joint_group->getEndEffectorTips(ik_tips);
    for (const auto * end_effector : model->getEndEffectors()) {
      const auto & tip = end_effector->getEndEffectorParentGroup().second;
      if (joint_group->hasLinkModel(tip) &&
        std::find(ik_tips.begin(), ik_tips.end(), tip) == ik_tips.end())
      {
        ik_tips.push_back(tip);
      }
    }
    for (const auto & requested_tip : requested_tips) {
      if (std::find(ik_tips.begin(), ik_tips.end(), requested_tip) == ik_tips.end()) {
        ik_tips.push_back(requested_tip);
      }
    }

    auto ik_request = std::make_shared<moveit_msgs::srv::GetPositionIK::Request>();
    ik_request->ik_request.group_name = move_group->getName();

    // Ask MoveIt to reject IK states that collide with the current planning scene.
    ik_request->ik_request.avoid_collisions = true;
    ik_request->ik_request.timeout = rclcpp::Duration::from_seconds(ik_timeout_);

    // The current full robot state is the IK seed and supplies values for joints outside the
    // selected group, including the mobile base when planning only the arms.
    moveit::core::robotStateToRobotStateMsg(
      *target_state, ik_request->ik_request.robot_state, true);

    for (const auto & tip : ik_tips) {
      geometry_msgs::msg::PoseStamped target;
      target.header.stamp = service_node_->now();
      target.header.frame_id = move_group->getPlanningFrame();
      const auto requested = requested_targets.find(tip);

      // A multi-tip solver expects one pose for each configured tip. Hold an omitted arm at its
      // current pose so callers may still target only left or right using a composite group.
      target.pose = requested == requested_targets.end() ?
        pose_from_state(*start_state, tip) : requested->second;
      ik_request->ik_request.ik_link_names.push_back(tip);
      ik_request->ik_request.pose_stamped_vector.push_back(target);
    }

    // The executor must remain multi-threaded: another thread receives this asynchronous reply
    // while the current service callback waits on the future.
    auto future = compute_ik_client_->async_send_request(ik_request);
    const auto wait_result = future.wait_for(
      std::chrono::duration<double>(ik_timeout_ + 2.0));
    if (wait_result != std::future_status::ready) {
      set_response(response, false, "Timed out waiting for MoveIt's IK response");
      return false;
    }

    const auto ik_response = future.get();
    if (ik_response->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      set_response(
        response, false, "No IK solution for group '" + move_group->getName() +
        "' (MoveIt error " + std::to_string(ik_response->error_code.val) +
        "); verify the target poses and kinematics solver configuration");
      return false;
    }
    if (!moveit::core::robotStateMsgToRobotState(ik_response->solution, *target_state, true)) {
      set_response(response, false, "MoveIt returned an invalid IK robot state");
      return false;
    }
    // Convert the collision-checked Cartesian solution into a joint goal. OMPL now samples a
    // normal joint-space goal instead of trying to sample multiple pose constraints.
    target_state->update();
    if (!target_state->satisfiesBounds(joint_group)) {
      set_response(response, false, "The IK solution violates joint bounds");
      return false;
    }
    if (!move_group->setJointValueTarget(*target_state)) {
      set_response(response, false, "MoveIt rejected the IK joint-state target");
      return false;
    }

    // Produce the time-parameterized collision-free joint trajectory without
    // executing it; callers choose standard execution or task-space streaming.
    const auto plan_result = move_group->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      set_response(
        response, false, "Cartesian planning failed: " +
        moveit::core::error_code_to_string(plan_result));
      move_group->clearPoseTargets();
      return false;
    }
    if (!has_trajectory(plan)) {
      set_response(response, false, "Cartesian planning produced an empty trajectory");
      move_group->clearPoseTargets();
      return false;
    }
    publish_display_trajectory(move_group, plan);
    move_group->clearPoseTargets();
    return true;
  }

  void handle_pose_request(
    const std::shared_ptr<Move3DPose::Request> request,
    std::shared_ptr<Move3DPose::Response> response)
  {
    // MoveGroupInterface is not thread-safe. Serialize planning and execution.
    std::lock_guard<std::mutex> lock(request_mutex_);
    std::shared_ptr<MoveGroup> move_group;
    MoveGroup::Plan plan;
    moveit::core::RobotStatePtr start_state;
    if (!plan_cartesian_request(request, response, move_group, plan, start_state)) {
      return;
    }

    const auto execute_result = move_group->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      set_response(
        response, false, "Cartesian execution failed: " +
        moveit::core::error_code_to_string(execute_result));
      return;
    }
    set_response(
      response, true, "Cartesian motion for group '" + move_group->getName() + "' completed");
  }

  bool sample_joint_trajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const moveit::core::RobotState & start_state,
    double sample_time,
    std::vector<double> & positions,
    std::vector<double> & velocities) const
  {
    if (trajectory.joint_names.empty() || trajectory.points.empty()) {
      return false;
    }
    const double final_time =
      rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
    const bool hold_final = sample_time >= final_time;

    // Locate the first waypoint at or after the requested trajectory time.
    std::size_t upper_index = 0;
    while (upper_index < trajectory.points.size() &&
      rclcpp::Duration(trajectory.points[upper_index].time_from_start).seconds() < sample_time)
    {
      ++upper_index;
    }
    if (upper_index >= trajectory.points.size()) {
      upper_index = trajectory.points.size() - 1;
    }

    const auto & upper = trajectory.points[upper_index];
    if (upper.positions.size() != trajectory.joint_names.size()) {
      return false;
    }
    positions = upper.positions;
    velocities.assign(trajectory.joint_names.size(), 0.0);

    const double upper_time = rclcpp::Duration(upper.time_from_start).seconds();
    if (upper_index == 0) {
      // Interpolate from the measured start state when the first point is later than t=0.
      if (upper_time <= 1.0e-9) {
        if (interpolate_velocity_ &&
          upper.velocities.size() == trajectory.joint_names.size())
        {
          velocities = upper.velocities;
        }
        if (hold_final) {
          std::fill(velocities.begin(), velocities.end(), 0.0);
        }
        return true;
      }
      const double sample = std::clamp(sample_time, 0.0, upper_time);
      const double alpha = sample / upper_time;
      for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
        const double start_position =
          start_state.getVariablePosition(trajectory.joint_names[index]);
        const double delta = upper.positions[index] - start_position;
        if (interpolate_velocity_ &&
          upper.velocities.size() == trajectory.joint_names.size())
        {
          // Match joint_trajectory_controller's cubic Hermite rule when endpoint velocity exists.
          const double end_velocity = upper.velocities[index];
          const double s2 = alpha * alpha;
          const double s3 = s2 * alpha;
          positions[index] = (2.0 * s3 - 3.0 * s2 + 1.0) * start_position +
            (-2.0 * s3 + 3.0 * s2) * upper.positions[index] +
            (s3 - s2) * upper_time * end_velocity;
          velocities[index] =
            ((6.0 * s2 - 6.0 * alpha) * start_position +
            (-6.0 * s2 + 6.0 * alpha) * upper.positions[index]) / upper_time +
            (3.0 * s2 - 2.0 * alpha) * end_velocity;
        } else {
          positions[index] = start_position + alpha * delta;
          // Linear-only mode intentionally disables velocity feed-forward.
          velocities[index] = 0.0;
        }
      }
      if (hold_final) {
        std::fill(velocities.begin(), velocities.end(), 0.0);
      }
      return true;
    }

    // Sample each timed segment with either controller-style cubic interpolation or a linear
    // position ramp. Both modes derive q and q_dot from the same polynomial.
    const auto & lower = trajectory.points[upper_index - 1];
    if (lower.positions.size() != trajectory.joint_names.size()) {
      return false;
    }
    const double lower_time = rclcpp::Duration(lower.time_from_start).seconds();
    const double interval = upper_time - lower_time;
    if (interval <= 1.0e-9) {
      return true;
    }
    const double segment_time = std::clamp(sample_time - lower_time, 0.0, interval);
    const double alpha = segment_time / interval;
    for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
      if (interpolate_velocity_ &&
        lower.velocities.size() == trajectory.joint_names.size() &&
        upper.velocities.size() == trajectory.joint_names.size())
      {
        const double p0 = lower.positions[index];
        const double p1 = upper.positions[index];
        const double v0 = lower.velocities[index];
        const double v1 = upper.velocities[index];
        // Position plus velocity selects cubic Hermite interpolation, matching the
        // joint_trajectory_controller spline rule for these two supplied interfaces.
        const double s2 = alpha * alpha;
        const double s3 = s2 * alpha;
        positions[index] = (2.0 * s3 - 3.0 * s2 + 1.0) * p0 +
          (s3 - 2.0 * s2 + alpha) * interval * v0 +
          (-2.0 * s3 + 3.0 * s2) * p1 +
          (s3 - s2) * interval * v1;
        velocities[index] =
          ((6.0 * s2 - 6.0 * alpha) * p0 +
          (-6.0 * s2 + 6.0 * alpha) * p1) / interval +
          (3.0 * s2 - 4.0 * alpha + 1.0) * v0 +
          (3.0 * s2 - 2.0 * alpha) * v1;
      } else {
        const double delta = upper.positions[index] - lower.positions[index];
        positions[index] = lower.positions[index] + alpha * delta;
        // Continue publishing the interpolated position while commanding zero q_dot.
        velocities[index] = 0.0;
      }
    }

    // Once the trajectory ends, use the controller-style hold state: final q and zero q_dot.
    if (hold_final) {
      std::fill(velocities.begin(), velocities.end(), 0.0);
    }
    return true;
  }

  geometry_msgs::msg::Pose pose_from_state(
    moveit::core::RobotState & state, const std::string & tip_link) const
  {
    // Joint updates invalidate MoveIt's cached link transforms. Refresh them before FK so a
    // freshly received current state cannot trigger getGlobalLinkTransform's dirty-cache assert.
    state.updateLinkTransforms();
    const Eigen::Isometry3d & transform = state.getGlobalLinkTransform(tip_link);
    const Eigen::Quaterniond orientation(transform.rotation());
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    pose.orientation.x = orientation.x();
    pose.orientation.y = orientation.y();
    pose.orientation.z = orientation.z();
    pose.orientation.w = orientation.w();
    return pose;
  }

  bool twist_from_group_velocity(
    moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * joint_group,
    const std::string & tip_link,
    const Eigen::VectorXd & joint_velocities,
    geometry_msgs::msg::Twist & twist) const
  {
    if (!joint_group ||
      joint_velocities.size() != static_cast<Eigen::Index>(joint_group->getVariableCount()) ||
      !joint_velocities.allFinite())
    {
      return false;
    }
    const auto * link = state.getRobotModel()->getLinkModel(tip_link);
    if (!link) {
      return false;
    }

    // MoveIt's getJacobian() rejects branched groups such as dual_arm. Evaluate the directional
    // FK derivative around q instead; this includes every shared and branch joint in the group.
    constexpr double derivative_step = 1.0e-4;
    const auto & variable_names = joint_group->getVariableNames();
    moveit::core::RobotState minus_state(state);
    moveit::core::RobotState plus_state(state);
    for (std::size_t index = 0; index < variable_names.size(); ++index) {
      const double position = state.getVariablePosition(variable_names[index]);
      const double offset = 0.5 * derivative_step * joint_velocities[index];
      minus_state.setVariablePosition(variable_names[index], position - offset);
      plus_state.setVariablePosition(variable_names[index], position + offset);
    }
    minus_state.updateLinkTransforms();
    plus_state.updateLinkTransforms();
    const Eigen::Isometry3d minus_transform = minus_state.getGlobalLinkTransform(link);
    const Eigen::Isometry3d plus_transform = plus_state.getGlobalLinkTransform(link);
    const Eigen::Vector3d linear_velocity =
      (plus_transform.translation() - minus_transform.translation()) / derivative_step;
    const Eigen::AngleAxisd rotation_delta(
      plus_transform.rotation() * minus_transform.rotation().transpose());
    const Eigen::Vector3d angular_velocity =
      rotation_delta.axis() * rotation_delta.angle() / derivative_step;
    if (!linear_velocity.allFinite() || !angular_velocity.allFinite()) {
      return false;
    }

    twist.linear.x = linear_velocity.x();
    twist.linear.y = linear_velocity.y();
    twist.linear.z = linear_velocity.z();
    twist.angular.x = angular_velocity.x();
    twist.angular.y = angular_velocity.y();
    twist.angular.z = angular_velocity.z();
    return true;
  }

  bool twist_from_state(
    moveit::core::RobotState & state,
    const moveit::core::JointModelGroup * joint_group,
    const std::string & tip_link,
    geometry_msgs::msg::Twist & twist) const
  {
    if (!joint_group) {
      return false;
    }
    // Copy q_dot in the exact group-variable order expected by the Jacobian. Looking up each
    // value by name also handles arbitrary ordering in the incoming JointState message.
    const auto & variable_names = joint_group->getVariableNames();
    Eigen::VectorXd joint_velocities(variable_names.size());
    {
      std::lock_guard<std::mutex> lock(joint_velocity_mutex_);
      for (std::size_t index = 0; index < variable_names.size(); ++index) {
        const auto velocity = measured_joint_velocities_.find(variable_names[index]);
        if (velocity == measured_joint_velocities_.end() || !is_finite(velocity->second)) {
          return false;
        }
        joint_velocities[index] = velocity->second;
      }
    }

    return twist_from_group_velocity(
      state, joint_group, tip_link, joint_velocities, twist);
  }

  bool publish_tracking_sample(
    const std::shared_ptr<MoveGroup> & move_group,
    const moveit::core::RobotState & start_state,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const rclcpp::Time & command_stamp,
    double sample_time,
    std::vector<geometry_msgs::msg::Pose> & desired_poses)
  {
    std::vector<double> positions;
    std::vector<double> velocities;
    if (!sample_joint_trajectory(
        trajectory, start_state, sample_time, positions, velocities))
    {
      return false;
    }

    // Reconstruct q_d(t), run FK, and publish synchronized Cartesian/posture targets.
    moveit::core::RobotState desired_state(start_state);
    for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
      desired_state.setVariablePosition(trajectory.joint_names[index], positions[index]);
    }
    desired_state.update();

    // Reorder the sampled trajectory q_dot into the planning group's Jacobian column order.
    const auto * joint_group = desired_state.getRobotModel()->getJointModelGroup(
      move_group->getName());
    if (!joint_group) {
      return false;
    }
    std::unordered_map<std::string, double> sampled_velocity_by_name;
    for (std::size_t index = 0; index < trajectory.joint_names.size(); ++index) {
      sampled_velocity_by_name[trajectory.joint_names[index]] = velocities[index];
    }
    const auto & group_variable_names = joint_group->getVariableNames();
    Eigen::VectorXd group_velocities(group_variable_names.size());
    for (std::size_t index = 0; index < group_variable_names.size(); ++index) {
      const auto velocity = sampled_velocity_by_name.find(group_variable_names[index]);
      if (velocity == sampled_velocity_by_name.end() || !is_finite(velocity->second)) {
        return false;
      }
      group_velocities[index] = velocity->second;
    }

    // During trajectory playback this is the scheduled sample time. During final-point holding,
    // the caller supplies the current ROS time so controllers do not reject the command as stale.
    desired_poses.clear();
    desired_poses.reserve(tracking_tip_links_.size());
    for (std::size_t index = 0; index < tracking_tip_links_.size(); ++index) {
      geometry_msgs::msg::PoseStamped target;
      target.header.stamp = command_stamp;
      target.header.frame_id = tracking_frame_;
      target.pose = pose_from_state(desired_state, tracking_tip_links_[index]);
      desired_poses.push_back(target.pose);
      tracking_pose_publishers_[index]->publish(target);

      geometry_msgs::msg::TwistStamped target_velocity;
      target_velocity.header = target.header;
      if (!twist_from_group_velocity(
          desired_state, joint_group, tracking_tip_links_[index], group_velocities,
          target_velocity.twist))
      {
        return false;
      }
      tracking_velocity_publishers_[index]->publish(target_velocity);
    }

    sensor_msgs::msg::JointState posture_target;
    posture_target.header.stamp = command_stamp;
    posture_target.name = trajectory.joint_names;
    posture_target.position = positions;
    posture_target.velocity = velocities;
    tracking_joint_publisher_->publish(posture_target);

    // A measured state keeps its observation time rather than borrowing the desired sample time.
    const auto current_state = move_group->getCurrentState(0.05);
    if (current_state) {
      const auto observation_stamp = service_node_->now();
      const auto * current_joint_group = current_state->getRobotModel()->getJointModelGroup(
        move_group->getName());
      for (std::size_t index = 0; index < tracking_tip_links_.size(); ++index) {
        geometry_msgs::msg::PoseStamped current;
        current.header.stamp = observation_stamp;
        current.header.frame_id = tracking_frame_;
        current.pose = pose_from_state(*current_state, tracking_tip_links_[index]);
        current_pose_publishers_[index]->publish(current);

        geometry_msgs::msg::TwistStamped current_velocity;
        current_velocity.header = current.header;
        if (twist_from_state(
            *current_state, current_joint_group, tracking_tip_links_[index],
            current_velocity.twist))
        {
          current_velocity_publishers_[index]->publish(current_velocity);
        } else {
          RCLCPP_WARN_THROTTLE(
            service_node_->get_logger(), *service_node_->get_clock(), 1000,
            "MoveL cannot compute current TCP velocity for '%s'; verify joint velocity feedback",
            tracking_tip_links_[index].c_str());
        }
      }
    } else {
      RCLCPP_WARN_THROTTLE(
        service_node_->get_logger(), *service_node_->get_clock(), 1000,
        "MoveL cannot publish current TCP poses because no current robot state is available");
    }
    return true;
  }

  bool measure_tracking_errors(
    const std::shared_ptr<MoveGroup> & move_group,
    const std::vector<geometry_msgs::msg::Pose> & desired_poses,
    std::vector<double> & position_errors,
    std::vector<double> & orientation_errors) const
  {
    const auto current_state = move_group->getCurrentState(0.2);
    if (!current_state || desired_poses.size() != tracking_tip_links_.size()) {
      return false;
    }

    position_errors.clear();
    orientation_errors.clear();
    for (std::size_t index = 0; index < tracking_tip_links_.size(); ++index) {
      const auto actual = pose_from_state(*current_state, tracking_tip_links_[index]);
      const auto & desired = desired_poses[index];
      const double dx = desired.position.x - actual.position.x;
      const double dy = desired.position.y - actual.position.y;
      const double dz = desired.position.z - actual.position.z;
      position_errors.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));

      const Eigen::Quaterniond actual_q(
        actual.orientation.w, actual.orientation.x,
        actual.orientation.y, actual.orientation.z);
      const Eigen::Quaterniond desired_q(
        desired.orientation.w, desired.orientation.x,
        desired.orientation.y, desired.orientation.z);
      const double dot = std::clamp(std::abs(actual_q.dot(desired_q)), 0.0, 1.0);
      orientation_errors.push_back(2.0 * std::acos(dot));
    }
    return true;
  }

  void handle_move_l_request(
    const std::shared_ptr<MoveL::Request> request,
    std::shared_ptr<MoveL::Response> response)
  {
    // A synchronous MoveL call owns the command stream until final tracking is verified.
    std::lock_guard<std::mutex> lock(request_mutex_);
    response->published_samples = 0U;
    response->planned_duration = 0.0;

    if (!is_finite(request->position_tolerance) ||
      !is_finite(request->orientation_tolerance) ||
      !is_finite(request->tracking_timeout) ||
      request->position_tolerance < 0.0 || request->orientation_tolerance < 0.0 ||
      request->tracking_timeout < 0.0)
    {
      set_response(
        response, false, "MoveL tolerances and tracking_timeout must be finite and nonnegative");
      return;
    }
    const double position_tolerance = request->position_tolerance > 0.0 ?
      request->position_tolerance : default_position_tolerance_;
    const double orientation_tolerance = request->orientation_tolerance > 0.0 ?
      request->orientation_tolerance : default_orientation_tolerance_;
    const double tracking_timeout = request->tracking_timeout > 0.0 ?
      request->tracking_timeout : default_tracking_timeout_;

    // Reuse the same collision-aware IK and global planning path as move_cartesian.
    auto pose_request = std::make_shared<Move3DPose::Request>();
    pose_request->planning_group = request->planning_group;
    pose_request->end_effector_names = request->end_effector_names;
    pose_request->ee_poses = request->ee_poses;
    pose_request->mode = request->mode;
    pose_request->relative_frame = request->relative_frame;
    std::shared_ptr<MoveGroup> move_group;
    MoveGroup::Plan plan;
    moveit::core::RobotStatePtr start_state;
    if (!plan_cartesian_request(pose_request, response, move_group, plan, start_state)) {
      return;
    }

    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if (move_group->getRobotModel()->getModelFrame() != tracking_frame_) {
      set_response(
        response, false, "MoveL FK frame '" + move_group->getRobotModel()->getModelFrame() +
        "' does not match tracking_frame '" + tracking_frame_ + "'");
      return;
    }
    for (const auto & tip : tracking_tip_links_) {
      if (!move_group->getRobotModel()->hasLinkModel(tip)) {
        set_response(response, false, "MoveL tracking tip '" + tip + "' is not in the model");
        return;
      }
    }

    // Refuse to report success when the operational-space controller is disconnected.
    if (require_tracking_subscribers_) {
      for (std::size_t index = 0; index < tracking_pose_publishers_.size(); ++index) {
        if (tracking_pose_publishers_[index]->get_subscription_count() == 0U) {
          set_response(
            response, false, "No subscriber on tracking topic '" +
            tracking_pose_topics_[index] + "'");
          return;
        }
        if (tracking_velocity_publishers_[index]->get_subscription_count() == 0U) {
          set_response(
            response, false, "No subscriber on tracking topic '" +
            tracking_velocity_topics_[index] + "'");
          return;
        }
      }
      if (tracking_joint_publisher_->get_subscription_count() == 0U) {
        set_response(
          response, false, "No subscriber on tracking topic '" + tracking_joint_topic_ + "'");
        return;
      }
    }

    const double planned_duration =
      rclcpp::Duration(trajectory.points.back().time_from_start).seconds();
    if (!is_finite(planned_duration) || planned_duration < 0.0) {
      set_response(response, false, "MoveL planned trajectory has an invalid duration");
      return;
    }
    response->planned_duration = planned_duration;
    const auto ros_clock = service_node_->get_clock();
    const auto publish_period = rclcpp::Duration::from_seconds(1.0 / tracking_publish_rate_);
    const auto trajectory_start = ros_clock->now();
    auto next_publish_time = trajectory_start;
    std::vector<geometry_msgs::msg::Pose> final_desired_poses;

    // Drive interpolation from the node clock so commands remain synchronized with /clock when
    // simulation runs slower than wall time, pauses, or advances in discrete physics steps.
    while (rclcpp::ok()) {
      const auto now = ros_clock->now();
      if (now < trajectory_start) {
        set_response(response, false, "ROS time moved backwards while publishing MoveL");
        return;
      }
      const double sample_time = std::min(planned_duration, (now - trajectory_start).seconds());
      const auto command_stamp =
        trajectory_start + rclcpp::Duration::from_seconds(sample_time);
      if (!publish_tracking_sample(
          move_group, *start_state, trajectory, command_stamp, sample_time,
          final_desired_poses))
      {
        set_response(
          response, false,
          "MoveL could not sample the trajectory or compute a target TCP velocity");
        return;
      }
      ++response->published_samples;

      if (sample_time >= planned_duration) {
        break;
      }

      // Skip obsolete samples after a forward time jump instead of bursting stale targets.
      next_publish_time = next_publish_time + publish_period;
      if (next_publish_time <= now) {
        next_publish_time = now + publish_period;
      }
      if (!ros_clock->sleep_until(next_publish_time)) {
        if (rclcpp::ok()) {
          set_response(response, false, "ROS time changed while publishing MoveL");
        }
        return;
      }
    }

    // Keep publishing the last trajectory point at the normal command rate until convergence.
    // Error measurement remains at 10 Hz because it performs another current-state lookup.
    const auto observation_period = rclcpp::Duration::from_seconds(0.1);
    const auto tracking_start = ros_clock->now();
    const auto tracking_deadline =
      tracking_start + rclcpp::Duration::from_seconds(tracking_timeout);
    auto next_publish_time_hold = tracking_start;
    auto next_observation_time = tracking_start;
    while (rclcpp::ok()) {
      const auto now = ros_clock->now();
      if (now < tracking_start) {
        set_response(response, false, "ROS time moved backwards while verifying MoveL tracking");
        return;
      }

      if (!publish_tracking_sample(
          move_group, *start_state, trajectory, now, planned_duration,
          final_desired_poses))
      {
        set_response(
          response, false,
          "MoveL could not sample the final waypoint or compute a target TCP velocity");
        return;
      }
      ++response->published_samples;

      if (now >= next_observation_time) {
        if (measure_tracking_errors(
            move_group, final_desired_poses,
            response->final_position_errors, response->final_orientation_errors))
        {
          const bool position_ok = std::all_of(
            response->final_position_errors.begin(), response->final_position_errors.end(),
            [position_tolerance](double error) {return error <= position_tolerance;});
          const bool orientation_ok = std::all_of(
            response->final_orientation_errors.begin(), response->final_orientation_errors.end(),
            [orientation_tolerance](double error) {return error <= orientation_tolerance;});
          if (position_ok && orientation_ok) {
            set_response(
              response, true, "MoveL planned " + std::to_string(planned_duration) +
              " seconds and converged after " +
              std::to_string((now - trajectory_start).seconds()) + " ROS seconds");
            return;
          }
        }
        next_observation_time = now + observation_period;
      }

      if (now >= tracking_deadline) {
        break;
      }
      // Use ROS-time sleeping for both real and simulated systems and skip missed periods.
      next_publish_time_hold = next_publish_time_hold + publish_period;
      if (next_publish_time_hold <= now) {
        next_publish_time_hold = now + publish_period;
      }
      const auto wake_time = next_publish_time_hold < tracking_deadline ?
        next_publish_time_hold : tracking_deadline;
      if (!ros_clock->sleep_until(wake_time)) {
        if (rclcpp::ok()) {
          set_response(response, false, "ROS time changed while verifying MoveL tracking");
        }
        return;
      }
    }

    std::ostringstream failure;
    failure << "MoveL published the full reference but final tracking did not converge within "
            << tracking_timeout << " ROS seconds";
    if (response->final_position_errors.empty()) {
      failure << "; no current robot state was available for the final error measurement";
    } else {
      failure << "; final TCP errors:";
      for (std::size_t index = 0; index < response->final_position_errors.size(); ++index) {
        failure << " " << tracking_tip_links_[index] << " position="
                << response->final_position_errors[index] << " m, orientation="
                << response->final_orientation_errors[index] << " rad";
      }
    }
    set_response(response, false, failure.str());
  }

  void handle_joint_request(
    const std::shared_ptr<MoveJointPose::Request> request,
    std::shared_ptr<MoveJointPose::Response> response)
  {
    // Serialize this path with Cartesian requests because both operate on cached MoveIt groups.
    std::lock_guard<std::mutex> lock(request_mutex_);

    if (request->mode != MoveJointPose::Request::ABSOLUTE &&
      request->mode != MoveJointPose::Request::RELATIVE)
    {
      set_response(response, false, "Invalid joint mode; use ABSOLUTE (0) or RELATIVE (1)");
      return;
    }
    if (request->joint_positions.empty()) {
      set_response(response, false, "joint_positions must not be empty");
      return;
    }
    if (!std::all_of(
        request->joint_positions.begin(), request->joint_positions.end(),
        [](double value) {return is_finite(value);}))
    {
      set_response(response, false, "Joint positions must all be finite");
      return;
    }

    std::string error;
    auto move_group = get_move_group(request->planning_group, error);
    if (!move_group) {
      set_response(response, false, error);
      return;
    }
    const auto model = move_group->getRobotModel();
    const auto * joint_group = model->getJointModelGroup(move_group->getName());
    if (!joint_group) {
      set_response(response, false, "Robot model does not contain the selected planning group");
      return;
    }

    bool target_is_valid = false;
    if (request->joint_names.empty()) {
      // Full-group form: values must follow MoveIt's variable order for the selected group.
      auto targets = request->joint_positions;
      const auto current = move_group->getCurrentJointValues();
      if (targets.size() != current.size()) {
        set_response(
          response, false, "Expected " + std::to_string(current.size()) +
          " joint values for group '" + move_group->getName() + "', received " +
          std::to_string(targets.size()));
        return;
      }
      if (request->mode == MoveJointPose::Request::RELATIVE) {
        for (std::size_t index = 0; index < targets.size(); ++index) {
          targets[index] += current[index];
        }
      }
      target_is_valid = move_group->setJointValueTarget(targets);
    } else {
      // Named form: update only the listed variables and leave all other joints unchanged.
      if (request->joint_names.size() != request->joint_positions.size()) {
        set_response(response, false, "joint_names and joint_positions must have equal length");
        return;
      }

      const auto & group_variables = joint_group->getVariableNames();
      const std::unordered_set<std::string> valid_variables(
        group_variables.begin(), group_variables.end());
      std::unordered_set<std::string> seen_variables;
      auto targets = request->joint_positions;
      const auto current_state = move_group->getCurrentState(2.0);
      if (!current_state) {
        set_response(response, false, "Timed out waiting for the current robot state");
        return;
      }
      for (std::size_t index = 0; index < request->joint_names.size(); ++index) {
        const auto & name = request->joint_names[index];
        if (valid_variables.count(name) == 0U) {
          set_response(
            response, false, "Joint variable '" + name + "' is not in group '" +
            move_group->getName() + "'");
          return;
        }
        if (!seen_variables.insert(name).second) {
          set_response(response, false, "Joint variable '" + name + "' appears more than once");
          return;
        }
        if (request->mode == MoveJointPose::Request::RELATIVE) {
          // Convert each requested offset into an absolute target before passing it to MoveIt.
          targets[index] += current_state->getVariablePosition(name);
        }
      }
      target_is_valid = move_group->setJointValueTarget(request->joint_names, targets);
    }

    if (!target_is_valid) {
      set_response(response, false, "Joint target violates the robot model bounds");
      return;
    }

    plan_and_execute(
      move_group, response, "Joint motion for group '" + move_group->getName() + "'");
  }

  void handle_group_state_request(
    const std::shared_ptr<MoveGroupState::Request> request,
    std::shared_ptr<MoveGroupState::Response> response)
  {
    // Named states are joint targets resolved from the active SRDF, so clients
    // do not need to duplicate the state's joint names and positions.
    std::lock_guard<std::mutex> lock(request_mutex_);
    if (request->group_state.empty()) {
      set_response(response, false, "group_state must not be empty");
      return;
    }

    std::string error;
    auto move_group = get_move_group(request->planning_group, error);
    if (!move_group) {
      set_response(response, false, error);
      return;
    }

    // A group_state is scoped to its SRDF group. Validate it explicitly to
    // distinguish a typo or wrong group from a generic planning failure.
    const auto named_targets = move_group->getNamedTargets();
    if (std::find(
        named_targets.begin(), named_targets.end(), request->group_state) ==
      named_targets.end())
    {
      std::ostringstream stream;
      stream << "Unknown group_state '" << request->group_state << "' for group '" <<
        move_group->getName() << "'. Available states: ";
      for (std::size_t index = 0; index < named_targets.size(); ++index) {
        stream << (index == 0 ? "" : ", ") << named_targets[index];
      }
      if (named_targets.empty()) {
        stream << "none";
      }
      set_response(response, false, stream.str());
      return;
    }

    // Remove pose goals left by Cartesian requests and resolve the named state to joint targets.
    // plan_and_execute captures and applies the measured start state immediately before planning.
    move_group->clearPoseTargets();
    if (!move_group->setNamedTarget(request->group_state)) {
      set_response(
        response, false, "MoveIt rejected group_state '" + request->group_state +
        "' for group '" + move_group->getName() + "'");
      return;
    }

    plan_and_execute(
      move_group, response, "Group state '" + request->group_state +
      "' for group '" + move_group->getName() + "'");
  }

  void handle_state_request(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/* request */,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    // Trigger has no group field, so this diagnostic reports the configured default group.
    std::lock_guard<std::mutex> lock(request_mutex_);
    std::string error;
    auto move_group = get_move_group(default_group_, error);
    if (!move_group) {
      response->success = false;
      response->message = error;
      return;
    }

    const auto names = move_group->getJointNames();
    const auto values = move_group->getCurrentJointValues();
    std::ostringstream stream;
    stream << "group=" << move_group->getName() << "; joints={";
    for (std::size_t index = 0; index < std::min(names.size(), values.size()); ++index) {
      stream << (index == 0 ? "" : ", ") << names[index] << ": " << values[index];
    }
    stream << "}";
    response->success = true;
    response->message = stream.str();
  }

  rclcpp::Node::SharedPtr moveit_node_;
  rclcpp::Node::SharedPtr service_node_;
  std::string default_group_;
  std::unordered_set<std::string> supported_groups_;
  double planning_time_{5.0};
  int planning_attempts_{10};
  double velocity_scale_{0.5};
  double acceleration_scale_{0.5};
  double ik_timeout_{0.25};
  std::string compute_ik_service_;
  std::string move_group_node_;
  std::string move_group_namespace_;
  std::string display_trajectory_topic_;
  std::vector<std::string> tracking_tip_links_;
  std::vector<std::string> display_ee_waypoint_topics_;
  double display_ee_waypoint_size_{0.02};
  std::vector<std::string> tracking_pose_topics_;
  std::vector<std::string> tracking_velocity_topics_;
  std::vector<std::string> current_pose_topics_;
  std::vector<std::string> current_velocity_topics_;
  std::string joint_state_topic_;
  std::string tracking_joint_topic_;
  std::string tracking_frame_;
  double tracking_publish_rate_{200.0};
  bool interpolate_velocity_{true};
  double default_position_tolerance_{0.01};
  double default_orientation_tolerance_{0.05};
  double default_tracking_timeout_{5.0};
  bool require_tracking_subscribers_{true};
  std::unordered_map<std::string, std::shared_ptr<MoveGroup>> move_groups_;
  std::mutex request_mutex_;
  mutable std::mutex joint_velocity_mutex_;
  std::unordered_map<std::string, double> measured_joint_velocities_;
  rclcpp::Client<moveit_msgs::srv::GetPositionIK>::SharedPtr compute_ik_client_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr
    display_trajectory_publisher_;
  std::vector<rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr>
  display_ee_waypoint_publishers_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
  tracking_pose_publishers_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>
  tracking_velocity_publishers_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>
  current_pose_publishers_;
  std::vector<rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr>
  current_velocity_publishers_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr tracking_joint_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscription_;
  rclcpp::Service<Move3DPose>::SharedPtr pose_service_;
  rclcpp::Service<MoveJointPose>::SharedPtr joint_service_;
  rclcpp::Service<MoveGroupState>::SharedPtr group_state_service_;
  rclcpp::Service<MoveL>::SharedPtr move_l_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr state_service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  try {
    auto server = std::make_shared<RobotMotionServer>("robot_motion_server", options);

    // Multiple threads are required because service callbacks synchronously wait for MoveIt
    // actions/services whose responses are processed through moveit_node_.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(server->get_moveit_node());
    executor.add_node(server->get_service_node());
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("robot_motion_server"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
