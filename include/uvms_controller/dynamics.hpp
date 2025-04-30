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


#ifndef UVMS_CONTROLLER__DYNAMICS_HPP_
#define UVMS_CONTROLLER__DYNAMICS_HPP_

#include <string>
#include <algorithm>
#include <casadi/casadi.hpp>
#include "uvms_controller/so_loader.hpp"
#include "controller_interface/controller_interface.hpp"
#include "uvms_interfaces/msg/command.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace casadi_uvms
{

    using CmdType = uvms_interfaces::msg::Command;
    class Dynamics
    {
    private:
        // Store the dynamics function for the whole body robot
        casadi_uvms::FunctionLoader fun_service;

        std::vector<DM> vehicle_simulate_argument;
        std::vector<DM> vehicle_sim;
        std::vector<double> vehicle_next_states;

        std::vector<DM> arm_simulate_argument;
        std::vector<DM> arm_sim;
        std::vector<double> arm_next_states;

        std::vector<double> arm_base_f_ext;

        std::vector<double> next_states;

        std::vector<DM> joint_q_arg;
        std::vector<DM> pid_argument;
        std::vector<DM> pid_command;

        DM depth;
        std::vector<DM> uv_G_argument;
        std::vector<DM> uv_g;

        std::vector<DM> uv_J_ned_argument;
        std::vector<DM> uv_J_ned;

        std::vector<DM> uv_J_REF_ned_argument;
        std::vector<DM> uv_J_REF_ned;

        casadi::DM arm_J_ned;
        casadi::DM arm_J_REF_ned;

        casadi::DM is_coupled;
        std::vector<casadi::DM> base_To;
        std::vector<casadi::DM> manipulator_parameters;
        std::vector<casadi::DM> vehicle_parameters;
        tf2::Quaternion q_orig_base;

        double time_seconds;
        std::vector<DM> arm_H_argument;
        std::vector<DM> arm_B_argument;

        std::vector<DM> vehicle_H_argument;
        std::vector<DM> vehicle_B_argument;

        std::vector<DM> arm_H_;
        casadi::DM inv_arm_H_;
        std::vector<DM> arm_B_;

        std::vector<DM> uv_H_;
        casadi::DM inv_uv_H_;
        std::vector<DM> uv_B_;

        std::vector<DM> arm_optimal_command;
        casadi::DM arm_optimal_control_params;
        std::vector<DM> arm_opt_control_argument;

        std::vector<DM> uv_optimal_command;
        casadi::DM uv_optimal_control_params;
        std::vector<DM> uv_opt_control_argument;

        std::vector<DM> arm_f_base_argument;
        std::vector<DM> base_force;
        std::vector<double> arm_noise;



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

            std::vector<double> current_velocity = std::vector<double>(11, 0);
            std::vector<double> prev_velocity = std::vector<double>(11, 0);
            std::vector<double> next_velocity = std::vector<double>(11, 0);
            std::vector<double> next_acceleration = std::vector<double>(11, 0);
            std::vector<double> force_input = std::vector<double>(11, 0);

            std::vector<double> model_p = std::vector<double>(12); // model parameters
            std::vector<double> flow_velocity = std::vector<double>(6, 0);
            std::vector<double> uvms_base_TF_;

            std::vector<double> payload_properties = std::vector<double>(4, 0);

            std::vector<int> poseSubscriber;
            std::vector<int> poseCommander;
            std::vector<int> velSubscriber;
            std::vector<int> accSubscriber;
            std::vector<int> velCommander;
            std::vector<int> accCommander;
            std::vector<int> effortCommander;
            std::vector<int> payloadSubscriber;
            std::vector<double> sum_ki_buffer = std::vector<double>(10, 0);

            std::vector<double> XF= std::vector<double>(10, 0);
            std::vector<double> VF= std::vector<double>(10, 0);
            std::vector<double> AF= std::vector<double>(10, 0);

            std::vector<double> Kp;
            std::vector<double> Ki;
            std::vector<double> Kd;
            std::vector<double> pid_commands = std::vector<double>(10, 0);
            std::vector<double> u_min;
            std::vector<double> u_max;
            std::vector<double> joint_min;
            std::vector<double> joint_max;
            bool grabber_open;
            bool initialised_real_state = true;
            double lyapunov_energy;
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
            const rclcpp::Time &time,
            const rclcpp::Duration &period,
            int &agent_id);

        void simulate(
            const rclcpp::Logger &logger,
            const rclcpp::Clock::SharedPtr &clock,
            const rclcpp::Time &time,
            const rclcpp::Duration &period,
            int &agent_id);
            
        std::string matrixToString(const casadi::Matrix<double>& mat);

        controller_interface::return_type pid_controller(
            std::shared_ptr<CmdType> &uvms_commands,
            const rclcpp::Logger &logger,
            const rclcpp::Clock::SharedPtr &clock,
            const rclcpp::Time &time,
            const rclcpp::Duration &period,
            int &agent_id);

        controller_interface::return_type optimal_controller(
            std::shared_ptr<CmdType> &uvms_commands,
            const rclcpp::Logger &logger,
            const rclcpp::Clock::SharedPtr &clock,
            const rclcpp::Time &time,
            const rclcpp::Duration &period,
            int &agent_id);

        controller_interface::return_type force_controller(
            std::shared_ptr<CmdType> &uvms_commands,
            const rclcpp::Logger &logger,
            const rclcpp::Clock::SharedPtr &clock,
            const rclcpp::Time &time,
            const rclcpp::Duration &period,
            int &agent_id);
        // Helper function to convert casadi::DM to std::string
        std::string dm_to_string(const casadi::DM& dm) const;
    };
} // namespace casadi_uvms

#endif // UVMS_CONTROLLER__DYNAMICS_HPP_
