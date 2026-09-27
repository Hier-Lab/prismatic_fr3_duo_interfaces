#pragma once

#include <algorithm>
#include <chrono>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <rclcpp/parameter_client.hpp>
#include <rclcpp/rclcpp.hpp>

namespace prismatic_fr3_duo_interfaces
{

inline bool has_local_robot_model(const rclcpp::Node::SharedPtr & node)
{
  for (const auto & name : {"robot_description", "robot_description_semantic"}) {
    if (!node->has_parameter(name)) {
      return false;
    }
    const auto parameter = node->get_parameter(name);
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING ||
      parameter.as_string().empty())
    {
      return false;
    }
  }
  return true;
}

inline void declare_or_replace_parameter(
  const rclcpp::Node::SharedPtr & node, const rclcpp::Parameter & parameter)
{
  if (node->has_parameter(parameter.get_name())) {
    const auto result = node->set_parameter(parameter);
    if (!result.successful) {
      throw std::runtime_error(
              "Unable to set local MoveIt parameter '" + parameter.get_name() +
              "': " + result.reason);
    }
  } else {
    node->declare_parameter(parameter.get_name(), parameter.get_parameter_value());
  }
}

inline void ensure_moveit_model_parameters(
  const rclcpp::Node::SharedPtr & node, const std::string & move_group_node,
  double timeout_seconds)
{
  // Explicitly supplied local model parameters take precedence over remote discovery.
  if (has_local_robot_model(node)) {
    RCLCPP_INFO(node->get_logger(), "Using locally supplied MoveIt robot model parameters");
    return;
  }
  if (move_group_node.empty()) {
    throw std::runtime_error(
            "move_group_node is empty and no local robot_description/SRDF was supplied");
  }
  if (timeout_seconds <= 0.0) {
    throw std::runtime_error("move_group_parameter_timeout must be greater than zero");
  }

  // Copy only the model, SRDF, and planning-limit tree. Kinematics stays in move_group so this
  // client does not create duplicate solver plugins.
  const auto timeout = std::chrono::duration<double>(timeout_seconds);
  auto client = std::make_shared<rclcpp::SyncParametersClient>(node, move_group_node);
  if (!client->wait_for_service(timeout)) {
    throw std::runtime_error(
            "Parameter services for MoveIt node '" + move_group_node +
            "' were unavailable after " + std::to_string(timeout_seconds) + " seconds");
  }

  const auto listed = client->list_parameters({"robot_description_planning"}, 100U, timeout);
  std::set<std::string> names(
    listed.names.begin(), listed.names.end());
  names.insert("robot_description");
  names.insert("robot_description_semantic");
  const std::vector<std::string> requested_names(names.begin(), names.end());
  const auto parameters = client->get_parameters(requested_names, timeout);

  bool found_description = false;
  bool found_semantic = false;
  std::size_t planning_parameter_count = 0U;
  for (const auto & parameter : parameters) {
    if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
      continue;
    }
    declare_or_replace_parameter(node, parameter);
    found_description = found_description ||
      (parameter.get_name() == "robot_description" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
      !parameter.as_string().empty());
    found_semantic = found_semantic ||
      (parameter.get_name() == "robot_description_semantic" &&
      parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING &&
      !parameter.as_string().empty());
    if (parameter.get_name().find("robot_description_planning.") == 0U) {
      ++planning_parameter_count;
    }
  }

  if (!found_description || !found_semantic) {
    throw std::runtime_error(
            "MoveIt node '" + move_group_node +
            "' must expose non-empty robot_description and robot_description_semantic parameters");
  }
  RCLCPP_INFO(
    node->get_logger(), "Loaded robot model and %zu planning parameter(s) from '%s'",
    planning_parameter_count, move_group_node.c_str());
}

}  // namespace prismatic_fr3_duo_interfaces
