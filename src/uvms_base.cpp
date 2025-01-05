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

namespace
{
  constexpr auto DEFAULT_TRANSFORM_TOPIC = "/tf";
} // namespace

namespace uvms_controller
{

  UvmsControllerBase::UvmsControllerBase()
      : controller_interface::ControllerInterface(),
        rt_command_ptr_(nullptr),
        uvms_command_subscriber_(nullptr),
        frame_transform_publisher_(nullptr),
        realtime_frame_transform_publisher_(nullptr)
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

    uvms_command_subscriber_ = get_node()->create_subscription<casadi_uvms::CmdType>(
        "~/uvms/commands", rclcpp::SystemDefaultsQoS(),
        [this](const casadi_uvms::CmdType::SharedPtr msg)
        { rt_command_ptr_.writeFromNonRT(msg); });

    RCLCPP_INFO(get_node()->get_logger(), "configure successful");

    // initialize transform publisher and message
    try
    {
      frame_transform_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
          DEFAULT_TRANSFORM_TOPIC, rclcpp::SystemDefaultsQoS());

      realtime_frame_transform_publisher_ =
          std::make_shared<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
              frame_transform_publisher_);

      auto &frame_transform_message = realtime_frame_transform_publisher_->msg_;
      frame_transform_message.transforms.resize(6);
    }
    catch (const std::exception &e)
    {
      fprintf(
          stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
          e.what());
      return CallbackReturn::ERROR;
    }

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
    rt_command_ptr_ = realtime_tools::RealtimeBuffer<std::shared_ptr<casadi_uvms::CmdType>>(nullptr);
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::return_type UvmsControllerBase::update(
      const rclcpp::Time &time, const rclcpp::Duration &period)
  {
    auto uvms_commands = rt_command_ptr_.readFromRT();

    controller_interface::return_type result = validate_uvms_commands((*uvms_commands), total_command_size, get_node()->get_logger(), get_node()->get_clock());

    if (result == controller_interface::return_type::ERROR)
    {
      return result;
    };

    model_dynamics.dt = period.seconds();

    for (auto &uvms : model_dynamics.uvms_world)
    {
      uvms.prev_position = uvms.current_position;
      uvms.prev_velocity = uvms.current_velocity;

      uvms.current_position = get_state_values(uvms.poseSubscriber, 12);
      uvms.current_velocity = get_state_values(uvms.velSubscriber, 11);

      if ((*uvms_commands)->command_type == "force")
      {
        result = model_dynamics.force_controller((*uvms_commands), get_node()->get_logger(), get_node()->get_clock(), uvms.id);
      };
      if (result == controller_interface::return_type::ERROR)
      {
        return result;
      };

      // if ((*uvms_commands)->command_type == "velocity")
      // {
      //   result = model_dynamics.velocity_controller((*uvms_commands), get_node()->get_logger(), get_node()->get_clock(), uvms.id);
      // };
      // if (result == controller_interface::return_type::ERROR)
      // {
      //   return result;
      // };

      if ((*uvms_commands)->command_type == "position")
      {
        result = model_dynamics.position_controller((*uvms_commands), get_node()->get_logger(), get_node()->get_clock(), uvms.id);
      };
      if (result == controller_interface::return_type::ERROR)
      {
        return result;
      };

      model_dynamics.simulate(get_node()->get_logger(), get_node()->get_clock(), uvms.id);

      std::pair<std::vector<DM>, DM> fw_result = model_dynamics.publish_forward_kinematics(get_node()->get_logger(), get_node()->get_clock(), uvms.id);
      
      std::vector<DM> T_i = fw_result.first;
      DM qned = fw_result.second;

      if (realtime_frame_transform_publisher_->trylock())
      {
        auto &transforms = realtime_frame_transform_publisher_->msg_.transforms;
        for (size_t i = 0; i < T_i.size(); ++i)
        {
          auto &transform = transforms[i];
          transform.header.stamp = time;
          transform.header.frame_id = "base_link";
          transform.child_frame_id = uvms.prefix + "joint_" + std::to_string(i);

          transform.transform.translation.x = T_i[i].nonzeros()[0];
          transform.transform.translation.y = T_i[i].nonzeros()[1];
          transform.transform.translation.z = T_i[i].nonzeros()[2];

          transform.transform.rotation.w = T_i[i].nonzeros()[3];
          transform.transform.rotation.x = T_i[i].nonzeros()[4];
          transform.transform.rotation.y = T_i[i].nonzeros()[5];
          transform.transform.rotation.z = T_i[i].nonzeros()[6];
        }

        // Base transform
        auto &base_transform = transforms[T_i.size()]; // Use the next index after T_i.size()
        base_transform.header.stamp = time;
        base_transform.header.frame_id = "base_link";
        base_transform.child_frame_id = uvms.prefix + "vehicle_base";

        // Access position from qned
        base_transform.transform.translation.x = qned.nonzeros()[0];
        base_transform.transform.translation.y = qned.nonzeros()[1];
        base_transform.transform.translation.z = qned.nonzeros()[2];

        base_transform.transform.rotation.w = qned.nonzeros()[3];
        base_transform.transform.rotation.x = qned.nonzeros()[4];
        base_transform.transform.rotation.y = qned.nonzeros()[5];
        base_transform.transform.rotation.z = qned.nonzeros()[6];

        realtime_frame_transform_publisher_->unlockAndPublish();
      };

      set_command_values(uvms.poseCommander, uvms.next_position, 12);
      set_command_values(uvms.velCommander, uvms.next_velocity, 11);
      set_command_values(uvms.effortCommander, uvms.force_input, 11);
    }

    return controller_interface::return_type::OK;
  }

  // Helper functions
  std::vector<double> UvmsControllerBase::get_state_values(const std::vector<int> &indices, std::size_t count)
  {
    std::vector<double> values(count);
    for (std::size_t i = 0; i < count; ++i)
    {
      values[i] = state_interfaces_[indices[i]].get_value();
    }
    return values;
  }

  void UvmsControllerBase::set_command_values(const std::vector<int> &indices, const std::vector<double> &values, std::size_t count)
  {
    for (std::size_t i = 0; i < count; ++i)
    {
      command_interfaces_[indices[i]].set_value(values[i]);
    }
  }

  controller_interface::return_type UvmsControllerBase::validate_uvms_commands(
      std::shared_ptr<casadi_uvms::CmdType> &uvms_commands,
      size_t expected_command_size,
      const rclcpp::Logger &logger,
      const rclcpp::Clock::SharedPtr &clock)
  {
    // Use logger and clock directly
    if (!uvms_commands)
    {
      RCLCPP_ERROR_THROTTLE(
          logger, *clock, 1000,
          "uvms commands not received");

      // Create a default command
      auto default_command = std::make_shared<casadi_uvms::CmdType>();
      default_command->input.data.resize(expected_command_size, 0.0);
      default_command->command_type = "force";
      uvms_commands = default_command;
    }

    // Ensure the command type is valid
    if (uvms_commands->command_type != "position" &&
        uvms_commands->command_type != "velocity" &&
        uvms_commands->command_type != "force")
    {
      last_command_type_ = "";
      RCLCPP_ERROR_THROTTLE(
          logger, *clock, 1000,
          "Unsupported command type: '%s'",
          uvms_commands->command_type.c_str());
      return controller_interface::return_type::ERROR;
    }

    // Log the command type every time it changes using RCLCPP_INFO_FUNCTION
    if (uvms_commands->command_type != last_command_type_)
    {
      RCLCPP_INFO(
          logger,
          "Received command type: '%s'",
          uvms_commands->command_type.c_str());
      last_command_type_ = uvms_commands->command_type;
    }

    if (uvms_commands->input.data.size() != expected_command_size)
    {
      RCLCPP_ERROR_THROTTLE(
          logger, *clock, 1000,
          "Command type '%s' expects %zu elements, but received %zu",
          uvms_commands->command_type.c_str(),
          expected_command_size,
          uvms_commands->input.data.size());

      return controller_interface::return_type::ERROR;
    }

    return controller_interface::return_type::OK;
  };
} // namespace uvms_controller
