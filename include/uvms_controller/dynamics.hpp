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

#ifndef UVMS_CONTROLLER__DYNAMICS_HPP_
#define UVMS_CONTROLLER__DYNAMICS_HPP_

#include <string>
#include <algorithm>
#include <casadi/casadi.hpp>
#include "uvms_controller/so_loader.hpp"
#include "controller_interface/controller_interface.hpp"
#include "uvms_interfaces/msg/command.hpp"


namespace casadi_uvms
{

    using CmdType = uvms_interfaces::msg::Command;
    class Dynamics
    {
    private:
        // Store the dynamics function for the whole body robot
        casadi_uvms::FunctionLoader fun_service;
        std::vector<DM> arm_simulate_argument;
        std::vector<DM> arm_sim;
        std::vector<DM> forward_pose;
        std::vector<DM> vehicle_simulate_argument;
        std::vector<DM> vehicle_sim;
        std::vector<DM> uvms_simulate_argument;
        std::vector<DM> uvms_sim;
        std::vector<DM> joint_q_arg;
        std::vector<DM> pid_argument;
        std::vector<DM> pid_command;
        std::vector<double> next_states;
        std::vector<DM> quaternion_states;
        std::vector<DM> euler_states;
        DM depth;
        std::vector<DM> uv_G_argument;
        std::vector<DM> uv_g;
        std::vector<DM> uv_J_argument;
        std::vector<DM> uv_J;
    public:
        struct Model
        {
            int id;
            std::string prefix;
            std::vector<double> pose_rot;
            std::vector<double> pose_trl;
            
            std::vector<double> current_position = std::vector<double>(12);
            std::vector<double> prev_position = std::vector<double>(12);
            std::vector<double> next_position = std::vector<double>(12);

            std::vector<double> current_velocity = std::vector<double>(11,0);
            std::vector<double> prev_velocity = std::vector<double>(11,0);
            std::vector<double> next_velocity = std::vector<double>(11,0);
            std::vector<double> force_input = std::vector<double>(11,0);

            std::vector<double> model_p = std::vector<double>(12); // model parameters
            std::vector<double> flow_velocity = std::vector<double>(6,0);
            std::vector<double> uvms_base_TF_;
            std::vector<int> poseSubscriber;
            std::vector<int> poseCommander;
            std::vector<int> velSubscriber;
            std::vector<int> accSubscriber;
            std::vector<int> velCommander;
            std::vector<int> accCommander;
            std::vector<int> effortCommander;
            std::vector<double> sum_ki_buffer = std::vector<double>(10,0);
            std::vector<double> XF;
            std::vector<double> VF;
            std::vector<double> Kp;
            std::vector<double> Ki;
            std::vector<double> Kd;
            std::vector<double> pid_commands  = std::vector<double>(10,0);
            std::vector<double> u_min;
            std::vector<double> u_max;
            bool grabber_open;
        };

        std::vector<Model> uvms_world{}; // vector for future enhancement

        double dt;
        Dynamics()
        {
            init_dynamics(); // Automatically call init_dynamics upon instantiation
        }

        void init_dynamics();

        std::pair<std::vector<DM>, DM> publish_forward_kinematics(
            const rclcpp::Logger &logger,
            const rclcpp::Clock::SharedPtr &clock,
            int &agent_id);

        void simulate(
            const rclcpp::Logger &logger,
            const rclcpp::Clock::SharedPtr &clock,
            int &agent_id);

        controller_interface::return_type pid_controller(
            std::shared_ptr<CmdType> &uvms_commands,
            const rclcpp::Logger &logger,
            const rclcpp::Clock::SharedPtr &clock,
            int &agent_id);

        controller_interface::return_type force_controller(
            std::shared_ptr<CmdType> &uvms_commands,
            const rclcpp::Logger &logger,
            const rclcpp::Clock::SharedPtr &clock,
            int &agent_id);

        std::vector<double> convertEulerToQuaternion(const double r, const double p, const double y);
        std::vector<double> convertQuaternionToEuler(const double w, const double x, const double y, const double z);
    };
} // namespace casadi_uvms

#endif // UVMS_CONTROLLER__DYNAMICS_HPP_
