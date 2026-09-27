#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/robot_model/robot_model.h>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include "prismatic_fr3_duo_interfaces/moveit_model_parameters.hpp"
#include "prismatic_fr3_duo_interfaces/srv/gripper_command.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using GripperCommand = prismatic_fr3_duo_interfaces::srv::GripperCommand;
using MoveGroup = moveit::planning_interface::MoveGroupInterface;

namespace
{
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

bool has_trajectory(const MoveGroup::Plan & plan)
{
  return !plan.trajectory_.joint_trajectory.points.empty() ||
         !plan.trajectory_.multi_dof_joint_trajectory.points.empty();
}
}  // namespace

class GripperServer
{
public:
  GripperServer(const std::string & node_name, const rclcpp::NodeOptions & options)
  {
    // Apply the same launch parameters to both nodes, including use_sim_time.
    moveit_node_ = std::make_shared<rclcpp::Node>("gripper_moveit", options);
    service_node_ = std::make_shared<rclcpp::Node>(node_name, options);

    // Reuse the model already served by move_group so this process remains robot-independent.
    move_group_node_ = declare_or_get<std::string>(
      moveit_node_, "move_group_node", "/move_group");
    move_group_namespace_ = declare_or_get<std::string>(
      moveit_node_, "move_group_namespace", "");
    const double model_parameter_timeout = declare_or_get<double>(
      moveit_node_, "move_group_parameter_timeout", 10.0);
    prismatic_fr3_duo_interfaces::ensure_moveit_model_parameters(
      moveit_node_, move_group_node_, model_parameter_timeout);

    const auto groups = declare_or_get<std::vector<std::string>>(
      moveit_node_, "gripper_groups", {"left_hand", "right_hand"});
    if (groups.empty()) {
      throw std::runtime_error("gripper_groups must not be empty");
    }
    configured_groups_.insert(groups.begin(), groups.end());
    planning_time_ = declare_or_get<double>(moveit_node_, "planning_time", 3.0);
    velocity_scale_ = declare_or_get<double>(moveit_node_, "velocity_scale", 0.5);
    acceleration_scale_ = declare_or_get<double>(moveit_node_, "acceleration_scale", 0.5);
    open_position_ = declare_or_get<double>(moveit_node_, "open_position", 0.035);
    closed_position_ = declare_or_get<double>(moveit_node_, "closed_position", 0.0);

    if (!std::isfinite(open_position_) || !std::isfinite(closed_position_)) {
      throw std::runtime_error("open_position and closed_position must be finite");
    }

    // Eager initialization gives semantic end-effector resolution access to the robot model and
    // fails at startup if a configured gripper group is invalid.
    for (const auto & group : groups) {
      const MoveGroup::Options options(group, "robot_description", move_group_namespace_);
      auto move_group = std::make_shared<MoveGroup>(moveit_node_, options);
      configure_move_group(move_group);
      move_groups_.emplace(group, move_group);
    }

    service_ = service_node_->create_service<GripperCommand>(
      "control_gripper", std::bind(&GripperServer::handle_request, this, _1, _2));

    RCLCPP_INFO(
      service_node_->get_logger(), "Gripper service ready for %zu group(s)", move_groups_.size());
  }

  rclcpp::Node::SharedPtr get_service_node() const {return service_node_;}
  rclcpp::Node::SharedPtr get_moveit_node() const {return moveit_node_;}

private:
  void configure_move_group(const std::shared_ptr<MoveGroup> & move_group) const
  {
    move_group->setPlanningTime(planning_time_);
    move_group->setNumPlanningAttempts(5);
    move_group->setMaxVelocityScalingFactor(velocity_scale_);
    move_group->setMaxAccelerationScalingFactor(acceleration_scale_);
    move_group->startStateMonitor();
  }

  bool resolve_group(
    const std::string & requested_name, std::string & group_name, std::string & error) const
  {
    if (requested_name.empty()) {
      error = "End-effector names must not be empty";
      return false;
    }
    if (configured_groups_.count(requested_name) != 0U) {
      group_name = requested_name;
      return true;
    }

    const auto model = move_groups_.begin()->second->getRobotModel();
    if (model->hasEndEffector(requested_name)) {
      group_name = model->getEndEffector(requested_name)->getName();
    } else if (model->hasLinkModel(requested_name)) {
      for (const auto * end_effector : model->getEndEffectors()) {
        if (end_effector->getEndEffectorParentGroup().second == requested_name) {
          group_name = end_effector->getName();
          break;
        }
      }
    }

    if (group_name.empty()) {
      error = "Unknown end effector, tip link, or gripper group '" + requested_name + "'";
      return false;
    }
    if (configured_groups_.count(group_name) == 0U) {
      error = "End effector '" + requested_name + "' resolves to disabled gripper group '" +
        group_name + "'";
      return false;
    }
    return true;
  }

  void respond(
    const std::shared_ptr<GripperCommand::Response> & response,
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

  bool set_command_target(
    const std::shared_ptr<MoveGroup> & move_group,
    const std::string & command, std::string & error) const
  {
    const auto * group = move_group->getRobotModel()->getJointModelGroup(move_group->getName());
    if (!group) {
      error = "Robot model does not contain gripper group '" + move_group->getName() + "'";
      return false;
    }

    // Prefer SRDF named states when provided. The numeric fallback keeps the service usable with
    // generated MoveIt configurations that omitted open/close group states.
    const auto & named_states = group->getDefaultStateNames();
    if (std::find(named_states.begin(), named_states.end(), command) != named_states.end()) {
      if (!move_group->setNamedTarget(command)) {
        error = "MoveIt rejected named target '" + command + "' for group '" +
          move_group->getName() + "'";
        return false;
      }
      return true;
    }

    auto target = move_group->getCurrentJointValues();
    if (target.empty()) {
      error = "Gripper group '" + move_group->getName() + "' has no controllable variables";
      return false;
    }
    std::fill(
      target.begin(), target.end(), command == "open" ? open_position_ : closed_position_);
    if (!move_group->setJointValueTarget(target)) {
      error = "Configured " + command + " position violates bounds for gripper group '" +
        move_group->getName() + "'";
      return false;
    }
    return true;
  }

  bool execute_command(
    const std::string & group_name, const std::string & command, std::string & error) const
  {
    const auto move_group = move_groups_.at(group_name);
    move_group->setStartStateToCurrentState();
    if (!set_command_target(move_group, command, error)) {
      return false;
    }

    MoveGroup::Plan plan;
    const auto plan_result = move_group->plan(plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "Planning '" + command + "' for '" + group_name + "' failed: " +
        moveit::core::error_code_to_string(plan_result);
      return false;
    }
    if (!has_trajectory(plan)) {
      error = "Planning '" + command + "' for '" + group_name +
        "' produced an empty trajectory";
      return false;
    }

    const auto execute_result = move_group->execute(plan);
    if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
      error = "Executing '" + command + "' for '" + group_name + "' failed: " +
        moveit::core::error_code_to_string(execute_result);
      return false;
    }
    return true;
  }

  void handle_request(
    const std::shared_ptr<GripperCommand::Request> request,
    std::shared_ptr<GripperCommand::Response> response)
  {
    std::lock_guard<std::mutex> lock(request_mutex_);

    if (request->end_effector_names.empty()) {
      respond(response, false, "At least one end effector is required");
      return;
    }
    if (request->end_effector_names.size() != request->commands.size()) {
      respond(response, false, "end_effector_names and commands must have equal length");
      return;
    }

    std::vector<std::pair<std::string, std::string>> operations;
    std::unordered_set<std::string> used_groups;
    for (std::size_t index = 0; index < request->end_effector_names.size(); ++index) {
      const auto & command = request->commands[index];
      if (command != "open" && command != "close") {
        respond(
          response, false, "Invalid command '" + command + "'; use 'open' or 'close'");
        return;
      }

      std::string group_name;
      std::string error;
      if (!resolve_group(request->end_effector_names[index], group_name, error)) {
        respond(response, false, error);
        return;
      }
      if (!used_groups.insert(group_name).second) {
        respond(response, false, "Each gripper may appear only once in a request");
        return;
      }
      operations.emplace_back(group_name, command);
    }

    for (std::size_t index = 0; index < operations.size(); ++index) {
      std::string error;
      if (!execute_command(operations[index].first, operations[index].second, error)) {
        if (index != 0U) {
          error += "; earlier gripper commands in this request already completed";
        }
        respond(response, false, error);
        return;
      }
    }

    std::ostringstream message;
    message << "Completed " << operations.size() << " gripper command(s)";
    respond(response, true, message.str());
  }

  rclcpp::Node::SharedPtr moveit_node_;
  rclcpp::Node::SharedPtr service_node_;
  std::unordered_set<std::string> configured_groups_;
  std::unordered_map<std::string, std::shared_ptr<MoveGroup>> move_groups_;
  double planning_time_{3.0};
  double velocity_scale_{0.5};
  double acceleration_scale_{0.5};
  double open_position_{0.035};
  double closed_position_{0.0};
  std::string move_group_node_;
  std::string move_group_namespace_;
  std::mutex request_mutex_;
  rclcpp::Service<GripperCommand>::SharedPtr service_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  try {
    auto server = std::make_shared<GripperServer>("gripper_server", options);
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(server->get_moveit_node());
    executor.add_node(server->get_service_node());
    executor.spin();
  } catch (const std::exception & exception) {
    RCLCPP_FATAL(rclcpp::get_logger("gripper_server"), "%s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
