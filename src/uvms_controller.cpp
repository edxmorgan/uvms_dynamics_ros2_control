// Copyright 2020 PAL Robotics S.L.
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

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace uvms_controller
{
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

    if (params_.joints.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    joints_ = params_.joints;
    for (const auto &joint_ : joints_)
    {
      RCLCPP_INFO(get_node()->get_logger(), "Joints Registered --> %s", joint_.c_str());
// ##############################################################################################################################
      if (params_.joints_map.at(joint_).dynamics_identifier.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'Dynamic Identifier' for joint %s parameter was empty", joint_.c_str());
        return controller_interface::CallbackReturn::ERROR;
      };

      uvms_dynamics_identifier_ = params_.joints_map.at(joint_).dynamics_identifier;
 
      RCLCPP_INFO(get_node()->get_logger(), "uvms_dynamics_identifier_ Registered --> %s", uvms_dynamics_identifier_.c_str());

// ##############################################################################################################################
      
      if (params_.joints_map.at(joint_).publish_pose_interface.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'pose_publish_interface_' for joint %s parameter was empty", joint_.c_str());
        return controller_interface::CallbackReturn::ERROR;
      };

      uvms_publish_pose_interface_ = params_.joints_map.at(joint_).publish_pose_interface;
      for (const auto &pose_publish_interface_ : uvms_publish_pose_interface_)
      {
        RCLCPP_INFO(get_node()->get_logger(), "uvms_publish_pose_interface Registered --> %s", pose_publish_interface_.c_str());
      }
// ##############################################################################################################################

      if (params_.joints_map.at(joint_).publish_velocity_interface.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'Pose Interface' for joint %s parameter was empty", joint_.c_str());
        return controller_interface::CallbackReturn::ERROR;
      };

      uvms_publish_velocity_interface_ = params_.joints_map.at(joint_).publish_velocity_interface;
      for (const auto &vel_interface_ : uvms_publish_velocity_interface_)
      {
        RCLCPP_INFO(get_node()->get_logger(), "uvms_publish_velocity_interface Registered --> %s", vel_interface_.c_str());
      }
// ##############################################################################################################################

      if (params_.joints_map.at(joint_).subscribe_pose_interface.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'pose_subscribe_interface' for joint %s parameter was empty", joint_.c_str());
        return controller_interface::CallbackReturn::ERROR;
      };

      uvms_subscribe_pose_interface_ = params_.joints_map.at(joint_).subscribe_pose_interface;
      for (const auto &pose_subscribe_interface : uvms_subscribe_pose_interface_)
      {
        RCLCPP_INFO(get_node()->get_logger(), "uvms_subscribe_pose_interface_ Registered --> %s", pose_subscribe_interface.c_str());
      }
// ##############################################################################################################################

      if (params_.joints_map.at(joint_).subscribe_velocity_interface.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'subscribe_velocity_interface_' for joint %s parameter was empty", joint_.c_str());
        return controller_interface::CallbackReturn::ERROR;
      };

      uvms_subscribe_velocity_interface_ = params_.joints_map.at(joint_).subscribe_velocity_interface;
      for (const auto &velocity_subscribe_interface_ : uvms_subscribe_velocity_interface_)
      {
        RCLCPP_INFO(get_node()->get_logger(), "uvms_subscribe_velocity_interface_ Registered --> %s", velocity_subscribe_interface_.c_str());
      }
// ##############################################################################################################################
      if (params_.joints_map.at(joint_).effort_interface.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'effort_interface' for joint %s parameter was empty", joint_.c_str());
        return controller_interface::CallbackReturn::ERROR;
      };

      uvms_effort_interface_ = params_.joints_map.at(joint_).effort_interface;

      RCLCPP_INFO(get_node()->get_logger(), "uvms_effort_interface_ Registered --> %s", uvms_effort_interface_.c_str());

// ##############################################################################################################################
    }

    RCLCPP_INFO(get_node()->get_logger(), "read_parameters successful******");
    return controller_interface::CallbackReturn::SUCCESS;
  }

} // namespace uvms_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    uvms_controller::UvmsController, controller_interface::ControllerInterface)
