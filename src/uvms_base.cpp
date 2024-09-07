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

    RCLCPP_INFO(get_node()->get_logger(), "number of agent : %zu ", n);
    RCLCPP_INFO(get_node()->get_logger(), "force input size of single agent : %zu ", force_input_size);

    // Resize to number of simulation worlds
    total_command_size = n * force_input_size;

    uvms_command_subscriber_ = get_node()->create_subscription<CmdType>(
        "~/uvms/commands", rclcpp::SystemDefaultsQoS(),
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
    auto &uvmsds_ = model_dynamics.uvms_world[0];

    RCLCPP_INFO(get_node()->get_logger(), "after simulation position vehcile (%u, %f, %f, %f, %f, %f, %f, %f)",
                model_dynamics.uvms_world[0].id,
                state_interfaces_[uvmsds_.poseSubscriber[0]].get_value(),
                state_interfaces_[uvmsds_.poseSubscriber[1]].get_value(),
                state_interfaces_[uvmsds_.poseSubscriber[2]].get_value(),
                state_interfaces_[uvmsds_.poseSubscriber[3]].get_value(),
                state_interfaces_[uvmsds_.poseSubscriber[4]].get_value(),
                state_interfaces_[uvmsds_.poseSubscriber[5]].get_value(),
                state_interfaces_[uvmsds_.poseSubscriber[6]].get_value());

    // no command received yet
    if (!uvms_commands || !(*uvms_commands))
    {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "uvms commands not recieved");
      return controller_interface::return_type::OK;
    };

    if ((*uvms_commands)->data.size() != total_command_size)
    {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "reference commands size (%zu) does not match number of command interfaces (%zu)",
          (*uvms_commands)->data.size(), total_command_size);

      return controller_interface::return_type::ERROR;
    };

    model_dynamics.dt = period.seconds();
    for (std::size_t j = 0; j < model_dynamics.uvms_world.size(); j++)
    {
      auto &uvms = model_dynamics.uvms_world[j];

      uvms.current_position = {
          state_interfaces_[uvms.poseSubscriber[0]].get_value(),
          state_interfaces_[uvms.poseSubscriber[1]].get_value(),
          state_interfaces_[uvms.poseSubscriber[2]].get_value(),
          state_interfaces_[uvms.poseSubscriber[3]].get_value(),
          state_interfaces_[uvms.poseSubscriber[4]].get_value(),
          state_interfaces_[uvms.poseSubscriber[5]].get_value(),
          state_interfaces_[uvms.poseSubscriber[6]].get_value(),
          state_interfaces_[uvms.poseSubscriber[7]].get_value(),
          state_interfaces_[uvms.poseSubscriber[8]].get_value(),
          state_interfaces_[uvms.poseSubscriber[9]].get_value(),
          state_interfaces_[uvms.poseSubscriber[10]].get_value()};

      uvms.current_velocity = {
          state_interfaces_[uvms.velSubscriber[0]].get_value(),
          state_interfaces_[uvms.velSubscriber[1]].get_value(),
          state_interfaces_[uvms.velSubscriber[2]].get_value(),
          state_interfaces_[uvms.velSubscriber[3]].get_value(),
          state_interfaces_[uvms.velSubscriber[4]].get_value(),
          state_interfaces_[uvms.velSubscriber[5]].get_value(),
          state_interfaces_[uvms.velSubscriber[6]].get_value(),
          state_interfaces_[uvms.velSubscriber[7]].get_value(),
          state_interfaces_[uvms.velSubscriber[8]].get_value(),
          state_interfaces_[uvms.velSubscriber[9]].get_value()};

      uvms.force_input.assign((*uvms_commands)->data.begin(), (*uvms_commands)->data.begin() + 10);

      uvms.flow_velocity = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      uvms.model_p = {1e-05, 1e-05, 1e-05, 1e-05, 3, 2, 1.8, 0.3, 3, 2, 1.8, 0.3};

      model_dynamics.decoupled_simulate(model_dynamics.uvms_world[j].id);

      command_interfaces_[uvms.poseCommander[0]].set_value(uvms.next_position[0]);
      command_interfaces_[uvms.poseCommander[1]].set_value(uvms.next_position[1]);
      command_interfaces_[uvms.poseCommander[2]].set_value(uvms.next_position[2]);
      command_interfaces_[uvms.poseCommander[3]].set_value(uvms.next_position[3]);
      command_interfaces_[uvms.poseCommander[4]].set_value(uvms.next_position[4]);
      command_interfaces_[uvms.poseCommander[5]].set_value(uvms.next_position[5]);
      command_interfaces_[uvms.poseCommander[6]].set_value(uvms.next_position[6]);

      command_interfaces_[uvms.poseCommander[7]].set_value(uvms.next_position[7]);
      command_interfaces_[uvms.poseCommander[8]].set_value(uvms.next_position[8]);
      command_interfaces_[uvms.poseCommander[9]].set_value(uvms.next_position[9]);
      command_interfaces_[uvms.poseCommander[10]].set_value(uvms.next_position[10]);

      command_interfaces_[uvms.velCommander[0]].set_value(uvms.next_velocity[0]);
      command_interfaces_[uvms.velCommander[1]].set_value(uvms.next_velocity[1]);
      command_interfaces_[uvms.velCommander[2]].set_value(uvms.next_velocity[2]);
      command_interfaces_[uvms.velCommander[3]].set_value(uvms.next_velocity[3]);
      command_interfaces_[uvms.velCommander[4]].set_value(uvms.next_velocity[4]);
      command_interfaces_[uvms.velCommander[5]].set_value(uvms.next_velocity[5]);

      command_interfaces_[uvms.velCommander[6]].set_value(uvms.next_velocity[6]);
      command_interfaces_[uvms.velCommander[7]].set_value(uvms.next_velocity[7]);
      command_interfaces_[uvms.velCommander[8]].set_value(uvms.next_velocity[8]);
      command_interfaces_[uvms.velCommander[9]].set_value(uvms.next_velocity[9]);

      command_interfaces_[uvms.effortCommander[0]].set_value(uvms.force_input[0]);
      command_interfaces_[uvms.effortCommander[1]].set_value(uvms.force_input[1]);
      command_interfaces_[uvms.effortCommander[2]].set_value(uvms.force_input[2]);
      command_interfaces_[uvms.effortCommander[3]].set_value(uvms.force_input[3]);
      command_interfaces_[uvms.effortCommander[4]].set_value(uvms.force_input[4]);
      command_interfaces_[uvms.effortCommander[5]].set_value(uvms.force_input[5]);

      command_interfaces_[uvms.effortCommander[6]].set_value(uvms.force_input[6]);
      command_interfaces_[uvms.effortCommander[7]].set_value(uvms.force_input[7]);
      command_interfaces_[uvms.effortCommander[8]].set_value(uvms.force_input[8]);
      command_interfaces_[uvms.effortCommander[9]].set_value(uvms.force_input[9]);
    };

    // uvms logic here
    return controller_interface::return_type::OK;
  }

} // namespace uvms_controller
