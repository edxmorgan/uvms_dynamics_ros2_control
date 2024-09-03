// Copyright 2024 Edward Morgan
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "uvms_controller/uvms_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace uvms_controller
{
  // Function to convert std::vector<double> to string
  std::string vectorToString(const std::vector<double> &vec)
  {
    std::ostringstream oss;
    for (const auto &val : vec)
    {
      oss << val << " ";
    }
    return oss.str();
  }

  UvmsController::UvmsController() : UvmsControllerBase() {}

  void UvmsController::declare_parameters()
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }

  controller_interface::CallbackReturn UvmsController::read_parameters()
  {
    if (!param_listener_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
      return controller_interface::CallbackReturn::ERROR;
    };
    params_ = param_listener_->get_params();


    if (params_.agent.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'agent' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };

    // if (params_.base_TF_translation.empty())
    // {
    //   RCLCPP_ERROR(get_node()->get_logger(), "'base_TF_translation' parameter was empty");
    //   return controller_interface::CallbackReturn::ERROR;
    // };

    // std::vector<double> base_tf_translation_ = params_.base_TF_translation;
    // std::string base_tf_translation_str = vectorToString(base_tf_translation_);
    // RCLCPP_INFO(get_node()->get_logger(), "base_tf_translation_ --> [%s]", base_tf_translation_str.c_str());

    // if (params_.base_TF_rotation.empty())
    // {
    //   RCLCPP_ERROR(get_node()->get_logger(), "'base_TF_rotation' parameter was empty");
    //   return controller_interface::CallbackReturn::ERROR;
    // };
    // std::vector<double> base_tf_rotation_ = params_.base_TF_rotation;
    // std::string base_tf_rotation_str = vectorToString(base_tf_rotation_);
    // RCLCPP_INFO(get_node()->get_logger(), "base_tf_rotation_ --> [%s]", base_tf_rotation_str.c_str());

    // // Reserve enough space to avoid multiple reallocations
    // uvms_base_TF_.reserve(6);
    // // Insert elements from base_tf_translation_ and base_tf_rotation_ into uvms_base_TF_
    // uvms_base_TF_.insert(uvms_base_TF_.end(), base_tf_translation_.begin(), base_tf_translation_.end());
    // uvms_base_TF_.insert(uvms_base_TF_.end(), base_tf_rotation_.begin(), base_tf_rotation_.end());

    // std::string base_TF_str = vectorToString(uvms_base_TF_);
    // RCLCPP_INFO(get_node()->get_logger(), "base_TF_ --> [%s]", base_TF_str.c_str());

    // if (params_.joints.empty())
    // {
    //   RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    //   return controller_interface::CallbackReturn::ERROR;
    // };
    // joints_ = params_.joints;

    // // Define the desired order of dynamics_identifier
    // std::unordered_map<std::string, int> dynamics_order = {
    //     {"floating_base", 0},
    //     {"joint0", 1},
    //     {"joint1", 2},
    //     {"joint2", 3},
    //     {"joint3", 4},
    //     {"endeffector", 5}};

    // // Sort joints_ based on the custom order of dynamics_identifier
    // std::sort(joints_.begin(), joints_.end(), [&](const std::string &a, const std::string &b)
    //           { return dynamics_order.at(params_.joints_map.at(a).dynamics_identifier) <
    //                    dynamics_order.at(params_.joints_map.at(b).dynamics_identifier); });
    // int state_index = 0;
    // int command_index = 0;

    // for (const auto &joint_ : joints_)
    // {
    //   RCLCPP_INFO(get_node()->get_logger(), "Joint --> %s", joint_.c_str());
    //   //
    //   // command interfaces
    //   // ##############################################################################################################################
    //   if (params_.joints_map.at(joint_).dynamics_identifier.empty())
    //   {
    //     RCLCPP_ERROR(get_node()->get_logger(), "'Dynamic Identifier' for joint %s parameter was empty", joint_.c_str());
    //     return controller_interface::CallbackReturn::ERROR;
    //   };

    //   std::string uvms_dynamics_identifier_ = params_.joints_map.at(joint_).dynamics_identifier;

    //   RCLCPP_INFO(get_node()->get_logger(), "uvms_dynamics_identifier_ --> %s", uvms_dynamics_identifier_.c_str());

    //   // ##############################################################################################################################

    //   if (params_.joints_map.at(joint_).pose_topic_interface.empty())
    //   {
    //     RCLCPP_ERROR(get_node()->get_logger(), "'pose_publish_interface_' for joint %s parameter was empty", joint_.c_str());
    //     return controller_interface::CallbackReturn::ERROR;
    //   };

    //  std::vector<std::string> uvms_pose_topic_interface_ = params_.joints_map.at(joint_).pose_topic_interface;
    //   for (const auto &pose_interface_ : uvms_pose_topic_interface_)
    //   {
    //     state_interface_types_.push_back(std::string(joint_) + "/" + std::string(pose_interface_));
    //     RCLCPP_INFO(get_node()->get_logger(), "uvms_pose_topic_interface Registered [%d] --> %s", state_index++, pose_interface_.c_str());
    //   }
    //   // ##############################################################################################################################

    //   if (params_.joints_map.at(joint_).velocity_topic_interface.empty())
    //   {
    //     RCLCPP_ERROR(get_node()->get_logger(), "'Pose Interface' for joint %s parameter was empty", joint_.c_str());
    //     return controller_interface::CallbackReturn::ERROR;
    //   };

    //   std::vector<std::string> uvms_velocity_topic_interface_ = params_.joints_map.at(joint_).velocity_topic_interface;
    //   for (const auto &vel_interface_ : uvms_velocity_topic_interface_)
    //   {
    //     state_interface_types_.push_back(std::string(joint_) + "/" + std::string(vel_interface_));
    //     RCLCPP_INFO(get_node()->get_logger(), "uvms_velocity_topic_interface Registered [%d] --> %s", state_index++, vel_interface_.c_str());
    //   }

    //   // ##############################################################################################################################

    //   // pose command interfaces
    //   if (params_.joints_map.at(joint_).pose_command_interface.empty())
    //   {
    //     RCLCPP_ERROR(get_node()->get_logger(), "'pose_command_interface' for joint %s parameter was empty", joint_.c_str());
    //     return controller_interface::CallbackReturn::ERROR;
    //   };

    //   std::vector<std::string> uvms_pose_command_interface_ = params_.joints_map.at(joint_).pose_command_interface;
    //   for (const auto &pose_command_interface_ : uvms_pose_command_interface_)
    //   {
    //     command_interface_types_.push_back(std::string(joint_) + "/" + std::string(pose_command_interface_));
    //     RCLCPP_INFO(get_node()->get_logger(), "uvms_pose_command_interface_ Registered [%d] --> %s", command_index++, pose_command_interface_.c_str());
    //   }

    //   // ##############################################################################################################################
    //   // velocity command interfaces
    //   if (params_.joints_map.at(joint_).velocity_command_interface.empty())
    //   {
    //     RCLCPP_ERROR(get_node()->get_logger(), "'velocity_command_interface' for joint %s parameter was empty", joint_.c_str());
    //     return controller_interface::CallbackReturn::ERROR;
    //   };

    //   std::vector<std::string> uvms_velocity_command_interface_ = params_.joints_map.at(joint_).velocity_command_interface;
    //   for (const auto &velocity_command_interface_ : uvms_velocity_command_interface_)
    //   {
    //     command_interface_types_.push_back(std::string(joint_) + "/" + std::string(velocity_command_interface_));
    //     RCLCPP_INFO(get_node()->get_logger(), "uvms_velocity_command_interface_ Registered [%d] --> %s", command_index++, velocity_command_interface_.c_str());
    //   }

    //   // ##############################################################################################################################
    //   // effort command interfaces
    //   if (params_.joints_map.at(joint_).effort_command_interface.empty())
    //   {
    //     RCLCPP_ERROR(get_node()->get_logger(), "'effort_command_interface' for joint %s parameter was empty", joint_.c_str());
    //     return controller_interface::CallbackReturn::ERROR;
    //   };
    //   std::vector<std::string> uvms_effort_command_interface_ = params_.joints_map.at(joint_).effort_command_interface;
    //   for (const auto &effort_command_interface_ : uvms_effort_command_interface_)
    //   {
    //     command_interface_types_.push_back(std::string(joint_) + "/" + std::string(effort_command_interface_));
    //     RCLCPP_INFO(get_node()->get_logger(), "uvms_effort_command_interface_ Registered [%d] --> %s", command_index++, effort_command_interface_.c_str());
    //   }

    //   // ##############################################################################################################################
    // }
    return controller_interface::CallbackReturn::SUCCESS;
  }

} // namespace uvms_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    uvms_controller::UvmsController, controller_interface::ControllerInterface)
