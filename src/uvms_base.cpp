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
      : controller_interface::ControllerInterface(),
        rt_command_ptr_(nullptr),
        uvms_command_subscriber_(nullptr)
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
      fun_service.usage_cplusplus_checks("test", "libtest.so", "UVMS Controller");
      fun_service.uvms_dynamics = fun_service.load_casadi_fun("uvms_stochastic_Alloc", "libUVMSnext.so");
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }
    // RCLCPP_INFO(get_node()->get_logger(), "on_init successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn UvmsControllerBase::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto ret = this->read_parameters();
    if (ret != controller_interface::CallbackReturn::SUCCESS)
    {
      return ret;
    }

    uvms_command_subscriber_ = get_node()->create_subscription<CmdType>(
        "~/uvms_commands", rclcpp::SystemDefaultsQoS(),
        [this](const CmdType::SharedPtr msg)
        { rt_command_ptr_.writeFromNonRT(msg); });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
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
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        ordered_interfaces;

    if (
        !controller_interface::get_ordered_interfaces(
            state_interfaces_, state_interface_types_, std::string(""), ordered_interfaces) ||
        state_interface_types_.size() != ordered_interfaces.size())
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Expected %zu state interfaces, got %zu",
          state_interface_types_.size(), ordered_interfaces.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn UvmsControllerBase::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // reset command buffer
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type UvmsControllerBase::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration &period)
  {
    auto uvms_commands = rt_command_ptr_.readFromRT();

    delta_seconds_ = period.seconds();

    // // no command received yet
    // if (!uvms_commands || !(*uvms_commands))
    // {
    //   auto cmd = std::make_shared<std_msgs::msg::Float64MultiArray>();
    //   cmd->data = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    //   *uvms_commands = cmd; // Dereference the pointer before assigning
    // }

    
    // if ((*uvms_commands)->data.size() != params_.joints_map.at(joints_[0]).effort_command_interface.size())
    // {
    //   RCLCPP_ERROR_THROTTLE(
    //       get_node()->get_logger(), *(get_node()->get_clock()), 1000,
    //       "command size (%zu) does not match number of interfaces (%zu)",
    //       (*uvms_commands)->data.size(), params_.joints_map.at(joints_[0]).effort_command_interface.size());
    //   return controller_interface::return_type::ERROR;
    // };

    // RCLCPP_INFO(get_node()->get_logger(), "uvms update (%d)", static_cast<int>(uvms_controller::vehicle::VehiclePositionTopic::POSITION_Z));

    // // for (auto index = 0ul; index < command_interfaces_.size(); ++index)
    // {
    //   command_interfaces_[index].set_value((*uvms_commands)->data[index]);
    // }

    // Extract the values from the state interfaces
    // uvms_x0_ = {
    //     state_interfaces_[0].get_value(),
    //     state_interfaces_[1].get_value(),
    //     state_interfaces_[2].get_value(),
    //     state_interfaces_[3].get_value(),
    //     state_interfaces_[4].get_value(),
    //     state_interfaces_[5].get_value(),
    //     state_interfaces_[6].get_value(),
    //     state_interfaces_[13].get_value(),
    //     state_interfaces_[15].get_value(),
    //     state_interfaces_[17].get_value(),
    //     state_interfaces_[19].get_value(),
    //     state_interfaces_[7].get_value(),
    //     state_interfaces_[8].get_value(),
    //     state_interfaces_[9].get_value(),
    //     state_interfaces_[10].get_value(),
    //     state_interfaces_[11].get_value(),
    //     state_interfaces_[12].get_value(),
    //     state_interfaces_[14].get_value(),
    //     state_interfaces_[16].get_value(),
    //     state_interfaces_[18].get_value(),
    //     state_interfaces_[20].get_value()};

    // uvms_u0_ = {
    //     (*uvms_commands)->data[0],
    //     (*uvms_commands)->data[1],
    //     (*uvms_commands)->data[2],
    //     (*uvms_commands)->data[3],
    //     (*uvms_commands)->data[4],
    //     (*uvms_commands)->data[5],
    //     (*uvms_commands)->data[6],
    //     (*uvms_commands)->data[7],
    //     (*uvms_commands)->data[8],
    //     (*uvms_commands)->data[9],
    //     (*uvms_commands)->data[10],
    //     (*uvms_commands)->data[11]};

    // uvms_vc_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    // uvms_params_ = {1e-5, 1e-5, 1e-5, 1e-6, 3, 2, 0.85, 0.065};
    // uvsm_arg_ = {DM(uvms_x0_), DM(uvms_u0_), DM(delta_seconds_), DM(uvms_vc_), DM(uvms_params_), DM(uvms_base_TF_),
    //                             casadi::DM::eye(21), casadi::DM::eye(14), casadi::DM::eye(21)};

    // uvms_dynamic_response_ = fun_service.uvms_dynamics(uvsm_arg_);
    // forward_dynamics_res_ = std::vector<double>(uvms_dynamic_response_.at(0));

    // state_interfaces_[0].set_value(forward_dynamics_res[0]);
    // state_interfaces_[1].set_value(forward_dynamics_res[1]);
    // state_interfaces_[2].set_value(forward_dynamics_res[2]);
    // state_interfaces_[3].set_value(forward_dynamics_res[3]);
    // state_interfaces_[4].set_value(forward_dynamics_res[4]);
    // state_interfaces_[5].set_value(forward_dynamics_res[5]);
    // state_interfaces_[6].set_value(forward_dynamics_res[6]);

    // state_interfaces_[13].set_value(forward_dynamics_res[7]);
    // state_interfaces_[15].set_value(forward_dynamics_res[8]);
    // state_interfaces_[17].set_value(forward_dynamics_res[9]);
    // state_interfaces_[19].set_value(forward_dynamics_res[10]);

    // state_interfaces_[7].set_value(forward_dynamics_res[11]);
    // state_interfaces_[8].set_value(forward_dynamics_res[12]);
    // state_interfaces_[9].set_value(forward_dynamics_res[13]);
    // state_interfaces_[10].set_value(forward_dynamics_res[14]);
    // state_interfaces_[11].set_value(forward_dynamics_res[15]);
    // state_interfaces_[12].set_value(forward_dynamics_res[16]);

    // state_interfaces_[14].set_value(forward_dynamics_res[17]);
    // state_interfaces_[16].set_value(forward_dynamics_res[18]);
    // state_interfaces_[18].set_value(forward_dynamics_res[19]);
    // state_interfaces_[20].set_value(forward_dynamics_res[20]);
    return controller_interface::return_type::OK;
  }

} // namespace uvms_controller
