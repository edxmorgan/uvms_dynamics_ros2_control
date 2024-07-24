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

#include "udwadia_kalaba_controller/udwadia_kalaba_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace udwadia_kalaba_controller
{
  ForwardCommandController::ForwardCommandController() : ForwardControllersBase() {}

  void ForwardCommandController::declare_parameters()
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }

  controller_interface::CallbackReturn ForwardCommandController::read_parameters()
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

    // if (params_.reference_interface_name.empty())
    // {
    //   RCLCPP_ERROR(get_node()->get_logger(), "'reference interface_name' parameter was empty");
    //   return controller_interface::CallbackReturn::ERROR;
    // };

    if (params_.command_interface_name.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'command interface_name' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };

    dof_ = params_.joints.size();
    RCLCPP_INFO(get_node()->get_logger(), "Number of controllable degrees of freedom --> %lu  ", dof_);

    command_interface_types_.reserve(dof_);
    for (const auto &joint_name : params_.joints)
    {
      command_interface_types_.push_back(joint_name + "/" + params_.command_interface_name);
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "Number of observable States --> %lu ", dof_ * params_.state_interface_names.size());
    state_interface_types_.reserve(dof_ * params_.state_interface_names.size());

    for (const auto &joint_name : params_.joints)
    {
      for (const auto &interface : params_.state_interface_names)
      {
        state_interface_types_.push_back(joint_name + "/" + interface);
      }
    }
    return controller_interface::CallbackReturn::SUCCESS;
  }

} // namespace udwadia_kalaba_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    udwadia_kalaba_controller::ForwardCommandController, controller_interface::ControllerInterface)
