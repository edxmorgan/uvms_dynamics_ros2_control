// Copyright (C) 2024 Edward Morgan
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Affero General Public License as published by
// the Free Software Foundation, either version 3 of the License, or (at your
// option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU Affero General Public License for more details.
//
// You should have received a copy of the GNU Affero General Public License
// along with this program. If not, see <https://www.gnu.org/licenses/>.


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
#include "uvms_controller/dynamics.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "realtime_tools/realtime_publisher.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace uvms_controller
{

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

    casadi_uvms::Dynamics model_dynamics;

    std::vector<std::string> joints_;
    std::vector<std::string> agents_;

    std::vector<std::string> command_interface_types_;
    std::vector<std::string> state_interface_types_;
    std::vector<std::string> last_command_type_;
    size_t n;

  private:
    // Helper functions
    tf2::Quaternion q_orig_joint;
    tf2::Quaternion q_orig_base;
    std::vector<double> get_state_values(const std::vector<int> &indices, std::size_t count);
    void set_command_values(const std::vector<int> &indices, const std::vector<double> &values, std::size_t count);
    controller_interface::return_type validate_uvms_commands(
        std::shared_ptr<casadi_uvms::CmdType> &uvms_commands,
        size_t expected_command_size,
        const rclcpp::Logger &logger,
        const rclcpp::Clock::SharedPtr &clock);

    realtime_tools::RealtimeBuffer<std::shared_ptr<casadi_uvms::CmdType>> rt_command_ptr_;
    rclcpp::Subscription<casadi_uvms::CmdType>::SharedPtr uvms_command_subscriber_;


    std::shared_ptr<rclcpp::Publisher<tf2_msgs::msg::TFMessage>> frame_transform_publisher_;
    std::shared_ptr<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>
        realtime_frame_transform_publisher_;

    // private attributes
    size_t force_input_size = casadi_uvms::Dynamics::Model().force_input.size();
    size_t total_command_size;
  };

} // namespace uvms_controller

#endif // UVMS_CONTROLLER__UVMS_BASE_HPP_
