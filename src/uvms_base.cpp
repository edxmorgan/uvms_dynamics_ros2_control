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

#include "udwadia_kalaba_controller/udwadia_controllers_base.hpp"

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

namespace udwadia_kalaba_controller
{
  ForwardControllersBase::ForwardControllersBase()
      : controller_interface::ControllerInterface(),
        rt_command_ptr_(nullptr),
        ref_subscriber_(nullptr)
  {
  }

  controller_interface::CallbackReturn ForwardControllersBase::on_init()
  {
    try
    {
      declare_parameters();
      // Print the CasADi version
      std::string casadi_version = CasadiMeta::version();
      RCLCPP_INFO(get_node()->get_logger(), "CasADi version: %s", casadi_version.c_str());
      RCLCPP_INFO(get_node()->get_logger(), "Testing casadi ready for operations");
      // Use CasADi's "external" to load the compiled dynamics functions
      dynamics_service.usage_cplusplus_checks("test", "libtest.so");
      dynamics_service.control_law = dynamics_service.load_casadi_fun("U_input", "libUke.so");
      dynamics_service.control_law_grad = dynamics_service.load_casadi_fun("U_grad", "libUkeGrad.so");
    }
    catch (const std::exception &e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ForwardControllersBase::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    auto ret = this->read_parameters();
    if (ret != controller_interface::CallbackReturn::SUCCESS)
    {
      return ret;
    }

    // topics QoS
    auto subscribers_qos = rclcpp::SystemDefaultsQoS();
    subscribers_qos.keep_last(1);
    subscribers_qos.best_effort();

    // Reference Subscriber
    ref_subscriber_ = get_node()->create_subscription<RefType>(
        "~/reference", subscribers_qos,
        [this](const RefType::SharedPtr msg)
        { rt_command_ptr_.writeFromNonRT(msg); });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration ForwardControllersBase::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration command_interfaces_config;
    command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    command_interfaces_config.names = command_interface_types_;

    return command_interfaces_config;
  }

  controller_interface::InterfaceConfiguration ForwardControllersBase::state_interface_configuration()
      const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;
    state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    state_interfaces_config.names = state_interface_types_;

    return state_interfaces_config;
    // return controller_interface::InterfaceConfiguration{
    //   controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::CallbackReturn ForwardControllersBase::on_activate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    //  check if we have all resources defined in the "points" parameter
    //  also verify that we *only* have the resources defined in the "points" parameter
    // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        ordered_interfaces;
    if (
        !controller_interface::get_ordered_interfaces(
            command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
        command_interface_types_.size() != ordered_interfaces.size())
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
          command_interface_types_.size(), ordered_interfaces.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    // reset command buffer if a ref came through callback when controller was inactive
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<RefType>>(nullptr);

    RCLCPP_INFO(get_node()->get_logger(), "activate successful");
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn ForwardControllersBase::on_deactivate(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    // reset command buffer
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<RefType>>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type ForwardControllersBase::update(
      const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
  {
    auto reference_state = rt_command_ptr_.readFromRT();

    // no command received yet
    if (!reference_state || !(*reference_state))
    {
      return controller_interface::return_type::OK;
    }

    if ((*reference_state)->data.size() != state_interfaces_.size())
    {
      RCLCPP_ERROR_THROTTLE(
          get_node()->get_logger(), *(get_node()->get_clock()), 1000,
          "reference states size (%zu) does not match number of interfaces (%zu)",
          (*reference_state)->data.size(), state_interfaces_.size());
      return controller_interface::return_type::ERROR;
    }
    // RCLCPP_INFO(get_node()->get_logger(), "updating commands successful 1");
    for (auto index = 0ul; index < command_interfaces_.size(); ++index)
    {

      // manipulator states
      // model state is in reverse

      // state_interfaces_[0].get_value() is the position endeffector
      // state_interfaces_[1].get_value() is the velocity endeffector

      std::vector<double> q = {state_interfaces_[8].get_value(),
                               state_interfaces_[6].get_value(),
                               state_interfaces_[4].get_value(),
                               state_interfaces_[2].get_value()};

      std::vector<double> q_dot = {state_interfaces_[9].get_value(),
                                   state_interfaces_[7].get_value(),
                                   state_interfaces_[5].get_value(),
                                   state_interfaces_[3].get_value()};

      std::vector<double> q_des = {(*reference_state)->data[8],
                                   (*reference_state)->data[6],
                                   (*reference_state)->data[4],
                                   (*reference_state)->data[2]};

      std::vector<double> q_dot_des = {(*reference_state)->data[9],
                                   (*reference_state)->data[7],
                                   (*reference_state)->data[5],
                                   (*reference_state)->data[3]};

      std::vector<double> q_ddot_des = {0.0, 0.0, 0.0, 0.0};
      std::vector<double> alpha = {1500.0, 1500.0, 1500.0, 1500.0};
      std::vector<double> gamma = {4000.0, 4000.0, 4000.0, 4000.0};
      std::vector<double> H = {0.0, 0.0, 0.0, 0.0};
      std::vector<double> I0 = {0.005, -0.001, 0.016, 0, 0, 0, 0.0001391, 0.00015849, 6.196e-05,
                                0.073563, -9.1e-05, -0.000734, 0, 0, 0, 8.677e-05, 0.00086823, 0.00089146,
                                0.017, -0.026, 0.002, 0, 0, 0, 4.18e-05, 2.83e-05, 4.502e-05,
                                -3e-05, -0.003, -0.098, 0, 0, 0, 0.00050787, 0.00053587, 7.6e-05};

      std::vector<DM> argfd = {DM(q), DM(q_dot), DM(q_des), DM(q_dot_des), DM(q_ddot_des), DM(alpha), DM(gamma), DM(H), DM(I0)};
      std::vector<DM> input = dynamics_service.control_law(argfd);
      std::vector<double> torque_input = std::vector<double>(input.at(0));
      // states_interfaces listed in [position, velocity, ... pattern]
      command_interfaces_[0].set_value(0);
      for (size_t i = 1; i < torque_input.size() + 1; ++i)
      {
        size_t j = torque_input.size() - i;

        // std::cout << "Iteration " << i << ": j = " << j << std::endl;
        double tau_in_electric = motor_control.torqueToCurrentMap(static_cast<int>(i), torque_input[j]);
        // std::cout << "Iteration " << i << ": j = " << j << "tau values" <<tau_in_electric<< std::endl;
        command_interfaces_[i].set_value(tau_in_electric);
      }

      // vehicle states

      // state_interfaces_[10].get_value();
      // state_interfaces_[11].get_value();

      // etc
    }

    return controller_interface::return_type::OK;
  }

} // namespace udwadia_kalaba_controller
