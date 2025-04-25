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


#include "uvms_controller/uvms_controller.hpp"

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <unordered_map>
#include <regex>
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace uvms_controller
{
  // Function to convert std::vector<double> to string
  std::string vectorToString(const std::vector<double> &vec)
  {
    std::ostringstream oss;
    for (const auto &val : vec)
    {
      oss << val << " ";
    }
    return oss.str();
  }

  UvmsController::UvmsController() : UvmsControllerBase() {}

  void UvmsController::declare_parameters()
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
  }

  controller_interface::CallbackReturn UvmsController::read_parameters()
  {
    if (!param_listener_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
      return controller_interface::CallbackReturn::ERROR;
    };
    params_ = param_listener_->get_params();

    if (params_.agents.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'agent' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    agents_ = params_.agents;

    if (params_.joints.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    };
    joints_ = params_.joints;

    int state_index = 0;
    int command_index = 0;
    int idx = 0;

    model_dynamics.uvms_world.reserve(agents_.size());
    for (const auto &agent_ : agents_)
    {
      casadi_uvms::Dynamics::Model uvms_agent;
      uvms_agent.id = idx;

      uvms_agent.u_min = {-1, -1, -5, -5, -5, -1, -2.83664, -0.629139, -0.518764, -0.54};
      uvms_agent.u_max = {1, 1, 5, 5, 5, 1, 2.83664, 0.629139, 0.518764, 0.54};

      RCLCPP_INFO(get_node()->get_logger(), "Agent --> %s", agent_.c_str());
      RCLCPP_INFO(get_node()->get_logger(), "Agent id--> %d", uvms_agent.id);
      ////////////////////////////////////////////////
      if (params_.agents_map.at(agent_).prefix.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'prefix' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
      };

      std::string agent_prefix = params_.agents_map.at(agent_).prefix;
      uvms_agent.prefix = agent_prefix;
      RCLCPP_INFO(get_node()->get_logger(), "Agent prefix --> %s", agent_prefix.c_str());
      ////////////////////////////////////////////

      if (params_.agents_map.at(agent_).base_TF_translation.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'base_TF_translation' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
      };

      std::vector<double> base_tf_translation_ = params_.agents_map.at(agent_).base_TF_translation;
      std::string base_tf_translation_str = vectorToString(base_tf_translation_);
      RCLCPP_INFO(get_node()->get_logger(), "base_tf_translation_ --> [%s]", base_tf_translation_str.c_str());

      if (params_.agents_map.at(agent_).base_TF_rotation.empty())
      {
        RCLCPP_ERROR(get_node()->get_logger(), "'base_TF_rotation' parameter was empty");
        return controller_interface::CallbackReturn::ERROR;
      };
      std::vector<double> base_tf_rotation_ = params_.agents_map.at(agent_).base_TF_rotation;
      std::string base_tf_rotation_str = vectorToString(base_tf_rotation_);
      RCLCPP_INFO(get_node()->get_logger(), "base_tf_rotation_ --> [%s]", base_tf_rotation_str.c_str());

      // Insert elements from base_tf_translation_ and base_tf_rotation_ into uvms_base_TF_
      uvms_agent.uvms_base_TF_.insert(uvms_agent.uvms_base_TF_.end(), base_tf_translation_.begin(), base_tf_translation_.end());
      uvms_agent.uvms_base_TF_.insert(uvms_agent.uvms_base_TF_.end(), base_tf_rotation_.begin(), base_tf_rotation_.end());

      std::string base_TF_str = vectorToString(uvms_agent.uvms_base_TF_);
      RCLCPP_INFO(get_node()->get_logger(), "base_TF_ --> [%s]", base_TF_str.c_str());

      for (const auto &joint_ : joints_)
      {
        // command interfaces
        // ##############################################################################################################################
        if (params_.joints_map.at(joint_).name.empty())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "'name' for joint %s parameter was empty", joint_.c_str());
          return controller_interface::CallbackReturn::ERROR;
        };

        std::string uvms_joint_name_ = std::regex_replace(
            params_.joints_map.at(joint_).name,
            std::regex("\\{prefix\\}"),
            agent_prefix);

        RCLCPP_INFO(get_node()->get_logger(), "%s --> %s", joint_.c_str(), uvms_joint_name_.c_str());

        // ##############################################################################################################################

        if (params_.joints_map.at(joint_).pose_topic_interface.empty())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "'pose_publish_interface_' for joint %s parameter was empty", joint_.c_str());
          return controller_interface::CallbackReturn::ERROR;
        };

        std::vector<std::string> uvms_pose_topic_interface_ = params_.joints_map.at(joint_).pose_topic_interface;
        for (const auto &pose_interface_ : uvms_pose_topic_interface_)
        {
          std::string interface_string = std::string(uvms_joint_name_) + "/" + std::string(pose_interface_);
          state_interface_types_.push_back(interface_string);
          uvms_agent.poseSubscriber.push_back(state_index);
          RCLCPP_INFO(get_node()->get_logger(), "uvms_pose_topic_interface Registered [%d] --> %s", state_index++, interface_string.c_str());
        }
        // ##############################################################################################################################

        if (params_.joints_map.at(joint_).velocity_topic_interface.empty())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "'velocity Interface' for joint %s parameter was empty", joint_.c_str());
          return controller_interface::CallbackReturn::ERROR;
        };

        std::vector<std::string> uvms_velocity_topic_interface_ = params_.joints_map.at(joint_).velocity_topic_interface;
        for (const auto &vel_interface_ : uvms_velocity_topic_interface_)
        {
          std::string interface_string = std::string(uvms_joint_name_) + "/" + std::string(vel_interface_);
          state_interface_types_.push_back(interface_string);
          uvms_agent.velSubscriber.push_back(state_index);
          RCLCPP_INFO(get_node()->get_logger(), "uvms_velocity_topic_interface Registered [%d] --> %s", state_index++, interface_string.c_str());
        }

        // ##############################################################################################################################
        if (params_.joints_map.at(joint_).acceleration_topic_interface.empty())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "'acceleration Interface' for joint %s parameter was empty", joint_.c_str());
          return controller_interface::CallbackReturn::ERROR;
        };

        std::vector<std::string> uvms_acclereration_topic_interface_ = params_.joints_map.at(joint_).acceleration_topic_interface;
        for (const auto &acc_interface_ : uvms_acclereration_topic_interface_)
        {
          std::string interface_string = std::string(uvms_joint_name_) + "/" + std::string(acc_interface_);
          state_interface_types_.push_back(interface_string);
          uvms_agent.accSubscriber.push_back(state_index);
          RCLCPP_INFO(get_node()->get_logger(), "uvms_accelertation_topic_interface Registered [%d] --> %s", state_index++, interface_string.c_str());
        }

        // ##############################################################################################################################

        // pose command interfaces
        if (params_.joints_map.at(joint_).pose_command_interface.empty())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "'pose_command_interface' for joint %s parameter was empty", joint_.c_str());
          return controller_interface::CallbackReturn::ERROR;
        };

        std::vector<std::string> uvms_pose_command_interface_ = params_.joints_map.at(joint_).pose_command_interface;
        for (const auto &pose_command_interface_ : uvms_pose_command_interface_)
        {
          std::string interface_string = std::string(uvms_joint_name_) + "/" + std::string(pose_command_interface_);
          command_interface_types_.push_back(interface_string);
          uvms_agent.poseCommander.push_back(command_index);
          RCLCPP_INFO(get_node()->get_logger(), "uvms_pose_command_interface_ Registered [%d] --> %s", command_index++, interface_string.c_str());
        }

        // ##############################################################################################################################
        // velocity command interfaces
        if (params_.joints_map.at(joint_).velocity_command_interface.empty())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "'velocity_command_interface' for joint %s parameter was empty", joint_.c_str());
          return controller_interface::CallbackReturn::ERROR;
        };

        std::vector<std::string> uvms_velocity_command_interface_ = params_.joints_map.at(joint_).velocity_command_interface;
        for (const auto &velocity_command_interface_ : uvms_velocity_command_interface_)
        {
          std::string interface_string = std::string(uvms_joint_name_) + "/" + std::string(velocity_command_interface_);
          command_interface_types_.push_back(interface_string);
          uvms_agent.velCommander.push_back(command_index);
          RCLCPP_INFO(get_node()->get_logger(), "uvms_velocity_command_interface_ Registered [%d] --> %s", command_index++, interface_string.c_str());
        }

        // ##############################################################################################################################
        // acceleration command interfaces
        if (params_.joints_map.at(joint_).acceleration_command_interface.empty())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "'acceleration_command_interface' for joint %s parameter was empty", joint_.c_str());
          return controller_interface::CallbackReturn::ERROR;
        };

        std::vector<std::string> uvms_acceleration_command_interface_ = params_.joints_map.at(joint_).acceleration_command_interface;
        for (const auto &acceleration_command_interface_ : uvms_acceleration_command_interface_)
        {
          std::string interface_string = std::string(uvms_joint_name_) + "/" + std::string(acceleration_command_interface_);
          command_interface_types_.push_back(interface_string);
          uvms_agent.accCommander.push_back(command_index);
          RCLCPP_INFO(get_node()->get_logger(), "uvms_acceleration_command_interface_ Registered [%d] --> %s", command_index++, interface_string.c_str());
        }

        // ##############################################################################################################################
        // effort command interfaces
        if (params_.joints_map.at(joint_).effort_command_interface.empty())
        {
          RCLCPP_ERROR(get_node()->get_logger(), "'effort_command_interface' for joint %s parameter was empty", joint_.c_str());
          return controller_interface::CallbackReturn::ERROR;
        };
        std::vector<std::string> uvms_effort_command_interface_ = params_.joints_map.at(joint_).effort_command_interface;
        for (const auto &effort_command_interface_ : uvms_effort_command_interface_)
        {
          std::string interface_string = std::string(uvms_joint_name_) + "/" + std::string(effort_command_interface_);
          command_interface_types_.push_back(interface_string);
          uvms_agent.effortCommander.push_back(command_index);
          RCLCPP_INFO(get_node()->get_logger(), "uvms_effort_command_interface_ Registered [%d] --> %s", command_index++, interface_string.c_str());
        }
        // ##############################################################################################################################
      }
      model_dynamics.uvms_world.push_back(uvms_agent);
      idx++;
      RCLCPP_INFO(get_node()->get_logger(), "uvms_base_TF_ count : %zu ", uvms_agent.uvms_base_TF_.size());
      RCLCPP_INFO(get_node()->get_logger(), "pose subscriber count : %zu ", uvms_agent.poseSubscriber.size());
      RCLCPP_INFO(get_node()->get_logger(), "pose commander count : %zu ", uvms_agent.poseCommander.size());
      RCLCPP_INFO(get_node()->get_logger(), "vel subscriber count : %zu ", uvms_agent.velSubscriber.size());
      RCLCPP_INFO(get_node()->get_logger(), "vel commander count : %zu ", uvms_agent.velCommander.size());
      RCLCPP_INFO(get_node()->get_logger(), "effort commander count : %zu ", uvms_agent.effortCommander.size());
    };
    n = model_dynamics.uvms_world.size();
    if (n != agents_.size())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "number of registered agent {%zu} does not match number of parameter agents {%zu}", n, agents_.size());
      return controller_interface::CallbackReturn::ERROR;
    }

    return controller_interface::CallbackReturn::SUCCESS;
  }

} // namespace uvms_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    uvms_controller::UvmsController, controller_interface::ControllerInterface)
