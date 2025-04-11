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
      frame_transform_message.transforms.resize(5);
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
      if (uvms.initialised_real_state)
      {
        // 1) Find the uvms in the vector that has id == 0 and prefix == robot_real_
        auto it = std::find_if(
            model_dynamics.uvms_world.begin(),
            model_dynamics.uvms_world.end(),
            [&](const casadi_uvms::Dynamics::Model &candidate)
            {
              return (candidate.id == 0 && candidate.prefix == "robot_real_");
            });

        // 2) If found, use it to update the current uvms
        if (it != model_dynamics.uvms_world.end())
        {
          RCLCPP_INFO(get_node()->get_logger(), " %s found real uvms object and initialised with real states into sim", uvms.prefix.c_str());
          uvms.prev_position = it->current_position;
          uvms.prev_velocity = it->current_velocity;

          uvms.current_position = get_state_values(it->poseSubscriber, 11);
          uvms.current_velocity = get_state_values(it->velSubscriber, 11);

          // // Randomize the x and y components of the current position
          // std::random_device rd;
          // std::mt19937 gen(rd());
          // std::uniform_real_distribution<> dis_x(-10.0, 10.0); // Range for x position
          // std::uniform_real_distribution<> dis_y(-10.0, 10.0); // Range for y position

          // uvms.current_position[0] = dis_x(gen);
          // uvms.current_position[1] = dis_y(gen);
        }
        else
        {
          RCLCPP_INFO(get_node()->get_logger(), " %s  did not find real uvms object. Real states refused to be initialised into sim", uvms.prefix.c_str());
          // log an error or provide fallback behavior
          uvms.prev_position = uvms.current_position;
          uvms.prev_velocity = uvms.current_velocity;

          uvms.current_position = get_state_values(uvms.poseSubscriber, 11);
          uvms.current_velocity = get_state_values(uvms.velSubscriber, 11);
        }
        uvms.initialised_real_state = false;

        RCLCPP_INFO(get_node()->get_logger(), "uvms pose size %lu", uvms.current_position.size());

        RCLCPP_INFO(get_node()->get_logger(), "uvms pose initialized with %f %f %f %f %f %f  %f %f %f %f %f",
                    uvms.current_position[0],
                    uvms.current_position[1],
                    uvms.current_position[2],

                    uvms.current_position[3],
                    uvms.current_position[4],
                    uvms.current_position[5],

                    uvms.current_position[6],
                    uvms.current_position[7],
                    uvms.current_position[8],
                    uvms.current_position[9],
                    uvms.current_position[10]);
      }
      else
      {
        uvms.prev_position = uvms.current_position;
        uvms.prev_velocity = uvms.current_velocity;

        uvms.current_position = get_state_values(uvms.poseSubscriber, 11);
        uvms.current_velocity = get_state_values(uvms.velSubscriber, 11);
      }

      if ((*uvms_commands)->command_type.at(uvms.id) == "force")
      {
        result = model_dynamics.force_controller((*uvms_commands), get_node()->get_logger(), get_node()->get_clock(), time, period, uvms.id);
      };
      if (result == controller_interface::return_type::ERROR)
      {
        return result;
      };

      if ((*uvms_commands)->command_type.at(uvms.id) == "pid")
      {
        result = model_dynamics.pid_controller((*uvms_commands), get_node()->get_logger(), get_node()->get_clock(), time, period, uvms.id);
      };
      if (result == controller_interface::return_type::ERROR)
      {
        return result;
      };

      if ((*uvms_commands)->command_type.at(uvms.id) == "optimal")
      {
        result = model_dynamics.optimal_controller((*uvms_commands), get_node()->get_logger(), get_node()->get_clock(), time, period, uvms.id);
      };
      if (result == controller_interface::return_type::ERROR)
      {
        return result;
      };

      // if (uvms.prefix != "robot_real_")
      // {
      // };
      model_dynamics.simulate(get_node()->get_logger(), get_node()->get_clock(), time, period, uvms.id);

      std::pair<std::vector<DM>, DM> fw_result = model_dynamics.publish_forward_kinematics(get_node()->get_logger(), get_node()->get_clock(), time, period, uvms.id);

      std::vector<DM> T_i = fw_result.first;
      DM qned = fw_result.second;

      if (realtime_frame_transform_publisher_->trylock())
      {
        auto &transforms = realtime_frame_transform_publisher_->msg_.transforms;
        for (size_t i = 0; i < T_i.size(); ++i)
        {
          auto &jointTransform = transforms[i];
          jointTransform.header.stamp = time;
          jointTransform.header.frame_id = uvms.prefix + "base_link";
          jointTransform.child_frame_id = uvms.prefix + "joint_" + std::to_string(i);

          jointTransform.transform.translation.x = T_i[i].nonzeros()[0];
          jointTransform.transform.translation.y = T_i[i].nonzeros()[1];
          jointTransform.transform.translation.z = T_i[i].nonzeros()[2];

          q_orig_joint.setW(T_i[i].nonzeros()[3]);
          q_orig_joint.setX(T_i[i].nonzeros()[4]);
          q_orig_joint.setY(T_i[i].nonzeros()[5]);
          q_orig_joint.setZ(T_i[i].nonzeros()[6]);

          jointTransform.transform.rotation = tf2::toMsg(q_orig_joint);
        }
        realtime_frame_transform_publisher_->unlockAndPublish();
      };

      set_command_values(uvms.poseCommander, uvms.next_position, 11);
      set_command_values(uvms.velCommander, uvms.next_velocity, 11);
      set_command_values(uvms.effortCommander, uvms.force_input, 11);
      set_command_values(uvms.accCommander, uvms.next_acceleration, 11);
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
    // If no commands were received, create a default one.
    if (!uvms_commands)
    {
      RCLCPP_ERROR_THROTTLE(
          logger, *clock, 1000,
          "uvms commands not received");

      // Create a default command
      auto default_command = std::make_shared<casadi_uvms::CmdType>();
      default_command->force.data.resize(expected_command_size, 0.0);
      // Initialize the command_type vector for each command with "force"
      default_command->command_type.resize(n, std::string("force"));
      uvms_commands = default_command;
    }

    if (uvms_commands->command_type.size() != n)
    {
      RCLCPP_ERROR_THROTTLE(
          logger, *clock, 1000,
          "number of controllers does not match the number of simulation robots.");
    }

    bool size_mismatch = false;
    std::string mismatched_field;
    std::size_t received_size = 0;

    // Validate each element in command_type.
    for (const auto &cmd : uvms_commands->command_type)
    {
      if (cmd != "pid" && cmd != "optimal" && cmd != "force")
      {
        // Clear the last command type if an unsupported command is encountered.
        last_command_type_.clear();
        RCLCPP_ERROR_THROTTLE(
            logger, *clock, 1000,
            "Unsupported command type: '%s'",
            cmd.c_str());
        return controller_interface::return_type::ERROR;
      }

      // Validate the numeric arrays based on the common command type.
      if (cmd == "force")
      {
        if (uvms_commands->force.data.size() != expected_command_size)
        {
          size_mismatch = true;
          mismatched_field = "force";
          received_size = uvms_commands->force.data.size();
        }
      }
      else if (cmd == "pid")
      {
        if (uvms_commands->pose.data.size() != expected_command_size)
        {
          size_mismatch = true;
          mismatched_field = "pose";
          received_size = uvms_commands->pose.data.size();
        }
      }
      else if (cmd == "optimal")
      {
        if (uvms_commands->pose.data.size() != expected_command_size)
        {
          size_mismatch = true;
          mismatched_field = "pose";
          received_size = uvms_commands->pose.data.size();
        }
        else if (uvms_commands->twist.data.size() != expected_command_size)
        {
          size_mismatch = true;
          mismatched_field = "twist";
          received_size = uvms_commands->twist.data.size();
        }
        else if (uvms_commands->acceleration.data.size() != expected_command_size)
        {
          size_mismatch = true;
          mismatched_field = "acceleration";
          received_size = uvms_commands->acceleration.data.size();
        }
      }
    }
    // Helper lambda to join vector<string> into a single string.
    auto join_vector = [](const std::vector<std::string> &vec, const std::string &sep = ", ") -> std::string
    {
      std::string result;
      for (size_t i = 0; i < vec.size(); ++i)
      {
        result += vec[i];
        if (i != vec.size() - 1)
        {
          result += sep;
        }
      }
      return result;
    };

    // Convert command_type vector to a string for logging.
    std::string command_type_str = join_vector(uvms_commands->command_type);

    // Log the command type if it has changed.
    if (uvms_commands->command_type != last_command_type_)
    {
      RCLCPP_INFO(
          logger,
          "Received list of command types: '[%s]'",
          command_type_str.c_str());
      last_command_type_ = uvms_commands->command_type;
    }

    if (size_mismatch)
    {
      RCLCPP_ERROR_THROTTLE(
          logger, *clock, 1000,
          "Command type '[%s]' expects %zu elements in '%s', but received %zu.",
          command_type_str.c_str(),
          expected_command_size,
          mismatched_field.c_str(),
          received_size);

      return controller_interface::return_type::ERROR;
    }
    // Return OK if no errors are found.
    return controller_interface::return_type::OK;
  }
} // namespace uvms_controller
