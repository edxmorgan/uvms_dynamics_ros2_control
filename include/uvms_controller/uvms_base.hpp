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
#include "uvms_controller/so_loader.hpp"

namespace uvms_controller
{

  using CmdType = std_msgs::msg::Float64MultiArray;

  class UvmsControllerBase : public controller_interface::ControllerInterface
  {
  public:
    UVMS_CONTROLLER_PUBLIC
    UvmsControllerBase();

    UVMS_CONTROLLER_PUBLIC
    ~UvmsControllerBase() = default;

    UVMS_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    UVMS_CONTROLLER_PUBLIC
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    UVMS_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_init() override;

    UVMS_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State &previous_state) override;

    UVMS_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State &previous_state) override;

    UVMS_CONTROLLER_PUBLIC
    controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State &previous_state) override;

    UVMS_CONTROLLER_PUBLIC
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

    // Store the dynamics function for the whole body robot
    casadi_uvms::FunctionLoader fun_service;

    std::vector<std::string> joints_;
    std::vector<double> uvms_base_TF_;  // Create a new vector to hold the combined tfs

    std::string uvms_dynamics_identifier_;
    std::vector<std::string> uvms_publish_pose_interface_;
    std::vector<std::string> uvms_publish_velocity_interface_;
    std::vector<std::string> uvms_subscribe_pose_interface_;
    std::vector<std::string> uvms_subscribe_velocity_interface_;
    std::vector<std::string> uvms_effort_command_interface_;

    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;

    realtime_tools::RealtimeBuffer<std::shared_ptr<CmdType>> rt_command_ptr_;
    rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;

    double delta_seconds_; // simulation period dt
    std::vector<double> uvms_x0_;           // current uvms state
    std::vector<double> uvms_u0_;           // uvms effort input
    std::vector<double> uvms_vc_;           // uvms current flow velocity . NB. irrotational assumed
    std::vector<double> uvms_params_;       // uvms manipulator parameters *{rotor_intial, viscous coefficient}
    std::vector<double> uvms_base_T_;       // vehicle base_origin to manipulator base_origin (x,y,z, r, p, y)
    std::vector<DM> uvsm_arg_;              // argument set for dynamics calculation
    std::vector<DM> uvms_dynamic_response_; // dynamic response
    std::vector<double> forward_dynamics_res_;   // stores the dynamic response from the forward dynamics simulator as double vector object
  };

} // namespace uvms_controller

#endif // UVMS_CONTROLLER__UVMS_BASE_HPP_
