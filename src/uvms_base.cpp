// Copyright 2021 Stogl Robotics Consulting UG (haftungsbescrhänkt)
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

#include "uvms_controller/uvms_base.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_interface/helpers.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

using namespace casadi;

namespace uvms_controller
{

  UvmsControllerBase::UvmsControllerBase()
      : controller_interface::ControllerInterface()
  {
  }

  controller_interface::CallbackReturn UvmsControllerBase::on_init()
  {
    try
    {
      declare_parameters();
      // Print the CasADi version
      std::string casadi_version = CasadiMeta::version();
      RCLCPP_INFO(get_node()->get_logger(), "UVMS Controller::CasADi version: %s", casadi_version.c_str());
      RCLCPP_INFO(get_node()->get_logger(), "UVMS Controller::Testing casadi ready for operations");
      // Use CasADi's "external" to load the compiled dynamics functions
      fun_service.usage_cplusplus_checks("test", "libtest.so","UVMS Controller");
      fun_service.dynamics = fun_service.load_casadi_fun("uvms_stochastic_Alloc", "libUVMSnext.so");
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn UvmsControllerBase::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration UvmsControllerBase::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names = command_interface_types_;

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration UvmsControllerBase::state_interface_configuration()
      const
  {
   controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = state_interface_types_;

    return state_interfaces_config;
  }

  controller_interface::CallbackReturn UvmsControllerBase::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn UvmsControllerBase::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type UvmsControllerBase::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
   // std::vector<double> uvms_x0 = {
    //     robot_structs_.hw_vehicle_struct_.current_state_.position_x,
    //     robot_structs_.hw_vehicle_struct_.current_state_.position_y,
    //     robot_structs_.hw_vehicle_struct_.current_state_.position_z,
    //     robot_structs_.hw_vehicle_struct_.current_state_.orientation_w,
    //     robot_structs_.hw_vehicle_struct_.current_state_.orientation_x,
    //     robot_structs_.hw_vehicle_struct_.current_state_.orientation_y,
    //     robot_structs_.hw_vehicle_struct_.current_state_.orientation_z,
    //     0.1, 0.2, 0.1, 0.2,
    //     robot_structs_.hw_vehicle_struct_.current_state_.u,
    //     robot_structs_.hw_vehicle_struct_.current_state_.v,
    //     robot_structs_.hw_vehicle_struct_.current_state_.w,
    //     robot_structs_.hw_vehicle_struct_.current_state_.p,
    //     robot_structs_.hw_vehicle_struct_.current_state_.q,
    //     robot_structs_.hw_vehicle_struct_.current_state_.r,
    //     0.0, 0.0, 0.0, 0.0};
    // std::vector<double> uvms_u0 = {
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[0].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[1].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[2].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[3].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[4].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[5].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[6].command_state_.effort,
    //     robot_structs_.hw_vehicle_struct_.hw_thrust_structs_[7].command_state_.effort, 0.0, 0.0, 0.0, 0.0};

    // std::vector<double> uvms_vc = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // std::vector<double> uvms_params = {1e-5, 1e-5, 1e-5, 1e-6, 3, 2, 0.85, 0.065};
    // std::vector<double> uvms_base_T = {0, 0, 0, 0, 0, 0};
    // std::vector<DM> uvsm_arg = {DM(uvms_x0), DM(uvms_u0), DM(delta_seconds), DM(uvms_vc), DM(uvms_params), DM(uvms_base_T),
    //                             casadi::DM::eye(21), casadi::DM::eye(14), casadi::DM::eye(21)};

    // std::vector<DM> uvms_dynamic_response = dynamics_service.uvms_dynamics(uvsm_arg);
    // forward_dynamics_res = std::vector<double>(uvms_dynamic_response.at(0));

    // robot_structs_.hw_vehicle_struct_.current_state_.position_x = forward_dynamics_res[0];
    // robot_structs_.hw_vehicle_struct_.current_state_.position_y = forward_dynamics_res[1];
    // robot_structs_.hw_vehicle_struct_.current_state_.position_z = forward_dynamics_res[2];
    // robot_structs_.hw_vehicle_struct_.current_state_.orientation_w = forward_dynamics_res[3];
    // robot_structs_.hw_vehicle_struct_.current_state_.orientation_x = forward_dynamics_res[4];
    // robot_structs_.hw_vehicle_struct_.current_state_.orientation_y = forward_dynamics_res[5];
    // robot_structs_.hw_vehicle_struct_.current_state_.orientation_z = forward_dynamics_res[6];

    // robot_structs_.hw_vehicle_struct_.current_state_.u = forward_dynamics_res[11];
    // robot_structs_.hw_vehicle_struct_.current_state_.v = forward_dynamics_res[12];
    // robot_structs_.hw_vehicle_struct_.current_state_.w = forward_dynamics_res[13];
    // robot_structs_.hw_vehicle_struct_.current_state_.p = forward_dynamics_res[14];
    // robot_structs_.hw_vehicle_struct_.current_state_.q = forward_dynamics_res[15];
    // robot_structs_.hw_vehicle_struct_.current_state_.r = forward_dynamics_res[16];
    return controller_interface::return_type::OK;
  }

} // namespace uvms_controller
