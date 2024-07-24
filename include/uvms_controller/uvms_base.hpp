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

#ifndef UVMS_CONTROLLER__UVMS_BASE_HPP_
#define UVMS_CONTROLLER__UVMS_BASE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <casadi/casadi.hpp>

#include "uvms_controller/visibility_control.h"
#include "uvms_controller/dynamics.hpp"

namespace uvms_controller
{
  using RefType = std_msgs::msg::Float64MultiArray;

  /**
   * \brief Udwadia Kalaba controller for a set of joints and interfaces.
   *
   * This class forwards the reference signal down to the control to compute command torques for set of joints or interfaces.
   *
   * Subscribes to:
   * - \b reference (std_msgs::msg::Float64MultiArray) : The reference to achieve.
   */
  class ForwardControllersBase : public controller_interface::ControllerInterface
  {
  public:
    UDWADIA_KALABA_CONTROLLER_PUBLIC
    ForwardControllersBase();

    UDWADIA_KALABA_CONTROLLER_PUBLIC
    ~ForwardControllersBase() = default;

    UDWADIA_KALABA_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    UDWADIA_KALABA_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    UDWADIA_KALABA_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    UDWADIA_KALABA_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    UDWADIA_KALABA_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    UDWADIA_KALABA_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    UDWADIA_KALABA_CONTROLLER_PUBLIC
    controller_interface::return_type update(
        const rclcpp::Time &time, const rclcpp::Duration &period) override;

  protected:
    /**
     * Derived controllers have to declare parameters in this method.
     * Error handling does not have to be done. It is done in `on_init`-method of this class.
     */
    virtual void declare_parameters() = 0;

    /**
     * Derived controllers have to read parameters in this method and set `command_interface_types_`
     * variable. The variable is then used to propagate the command interface configuration to
     * controller manager. The method is called from `on_configure`-method of this class.
     *
     * It is expected that error handling of exceptions is done.
     *
     * \returns controller_interface::CallbackReturn::SUCCESS if parameters are successfully read and
     * their values are allowed, controller_interface::CallbackReturn::ERROR otherwise.
     */
    virtual controller_interface::CallbackReturn read_parameters() = 0;

    size_t dof_;

    std::vector<std::string> joint_names_;
    std::string interface_name_;
    
    // Store the dynamics function for the robot joints
    casadi_reach_alpha_5::Dynamics dynamics_service;

    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;

    realtime_tools::RealtimeBuffer<std::shared_ptr<RefType>> rt_command_ptr_;
    rclcpp::Subscription<RefType>::SharedPtr ref_subscriber_;
  };

} // namespace udwadia_kalaba_controller

#endif // UVMS_CONTROLLER__UVMS_BASE_HPP_
