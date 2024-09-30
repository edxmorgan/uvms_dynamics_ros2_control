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

#include "uvms_controller/dynamics.hpp"

void casadi_uvms::Dynamics::init_dynamics()
{
    // Use CasADi's "external" to load the compiled dynamics functions
    fun_service.usage_cplusplus_checks("test", "libtest.so", "UVMS Controller");
    fun_service.decoupled_manipulator_uvms_dynamics = fun_service.load_casadi_fun("Mnext", "libMnext.so");
    fun_service.decoupled_vehicle_uvms_dynamics = fun_service.load_casadi_fun("Vnext", "libVnext.so");
    fun_service.manipulator_forward_kinematics = fun_service.load_casadi_fun("fkeval", "libFKeval.so");
    fun_service.vehicle_position_pid = fun_service.load_casadi_fun("pidC", "libPd.so");
    fun_service.vehicle_velocity_pid = fun_service.load_casadi_fun("vpidC", "libVPd.so");
};

controller_interface::return_type casadi_uvms::Dynamics::position_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr &clock,
    int &agent_id)
{
    std::vector<double> vehicle_pose_(uvms_world[agent_id].current_position.begin(),
                                      uvms_world[agent_id].current_position.begin() + 7);

    std::vector<double> vehicle_vel_(uvms_world[agent_id].current_velocity.begin(),
                                     uvms_world[agent_id].current_velocity.begin() + 6);

    std::vector<double> vehicle_state;
    vehicle_state.reserve(13);
    vehicle_state.insert(vehicle_state.end(), vehicle_pose_.begin(), vehicle_pose_.end());
    vehicle_state.insert(vehicle_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());

    uvms_world[agent_id].Kp = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    uvms_world[agent_id].Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    uvms_world[agent_id].Kd = {2, 2, 2, 2, 2, 2};

    int command_length_per_agent = 10; // Each agent's command contains 10 elements (position + quaternion + velocity)

    // Calculate the starting index for the current agent's data in the uvms_commands->input.data array
    int start_index = agent_id * command_length_per_agent;
    int end_index = start_index + 6;  // We are only interested in the first 6 elements (position + orientation)

    // Assign the first 6 elements (position + orientation) to uvms_world[agent_id].XF
    uvms_world[agent_id].XF.assign(uvms_commands->input.data.begin() + start_index, uvms_commands->input.data.begin() + end_index);

    vehicle_pose_pid_argument = {uvms_world[agent_id].Kp, uvms_world[agent_id].Ki, uvms_world[agent_id].Kd, uvms_world[agent_id].sum_ki_buffer, dt, vehicle_state, uvms_world[agent_id].XF};
    vehicle_pose_command = fun_service.vehicle_position_pid(vehicle_pose_pid_argument);

    std::vector<double> pid_commands = vehicle_pose_command.at(0).nonzeros();
    uvms_world[agent_id].sum_ki_buffer = vehicle_pose_command.at(1).nonzeros();
    std::copy(pid_commands.begin(), pid_commands.end(), uvms_world[agent_id].force_input.begin());
    return controller_interface::return_type::OK;
};

controller_interface::return_type casadi_uvms::Dynamics::velocity_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr &clock,
    int &agent_id)
{
    std::vector<double> vehicle_pose_(uvms_world[agent_id].current_position.begin(),
                                      uvms_world[agent_id].current_position.begin() + 7);

    std::vector<double> vehicle_vel_(uvms_world[agent_id].current_velocity.begin(),
                                     uvms_world[agent_id].current_velocity.begin() + 6);

    std::vector<double> vehicle_state;
    vehicle_state.reserve(13);
    vehicle_state.insert(vehicle_state.end(), vehicle_pose_.begin(), vehicle_pose_.end());
    vehicle_state.insert(vehicle_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());

    /////////////////////////////////////////////////////////////////////////////////
    std::vector<double> prev_vehicle_pose_(uvms_world[agent_id].prev_position.begin(),
                                           uvms_world[agent_id].prev_position.begin() + 7);

    std::vector<double> prev_vehicle_vel_(uvms_world[agent_id].prev_velocity.begin(),
                                          uvms_world[agent_id].prev_velocity.begin() + 6);

    std::vector<double> prev_vehicle_state;
    prev_vehicle_state.reserve(13);
    prev_vehicle_state.insert(prev_vehicle_state.end(), prev_vehicle_pose_.begin(), prev_vehicle_pose_.end());
    prev_vehicle_state.insert(prev_vehicle_state.end(), prev_vehicle_vel_.begin(), prev_vehicle_vel_.end());

    uvms_world[agent_id].Kp = {8.0, 8.0, 8.0, 8.0, 8.0, 8.0};
    uvms_world[agent_id].Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    uvms_world[agent_id].Kd = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1};

   int command_length_per_agent = 10; // Each agent's command contains 10 elements (position + quaternion + velocity)

    // Calculate the starting index for the current agent's data in the uvms_commands->input.data array
    int start_index = agent_id * command_length_per_agent;
    int end_index = start_index + 6;  // We are only interested in the first 6 elements (position + orientation)

    // Assign the first 6 elements (position + orientation) to uvms_world[agent_id].XF
    uvms_world[agent_id].XF.assign(uvms_commands->input.data.begin() + start_index, uvms_commands->input.data.begin() + end_index);

    vehicle_vel_pid_argument = {uvms_world[agent_id].Kp,
                                uvms_world[agent_id].Ki,
                                uvms_world[agent_id].Kd,
                                uvms_world[agent_id].sum_ki_buffer,
                                dt,
                                vehicle_state,
                                prev_vehicle_state,
                                uvms_world[agent_id].VF,
                                uvms_world[agent_id].pid_commands};
    vehicle_vel_command = fun_service.vehicle_velocity_pid(vehicle_vel_pid_argument);
    uvms_world[agent_id].pid_commands = vehicle_vel_command.at(0).nonzeros();
    uvms_world[agent_id].sum_ki_buffer = vehicle_vel_command.at(1).nonzeros();
    std::copy(uvms_world[agent_id].pid_commands.begin(), uvms_world[agent_id].pid_commands.end(), uvms_world[agent_id].force_input.begin());
    return controller_interface::return_type::OK;
};

controller_interface::return_type casadi_uvms::Dynamics::force_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr &clock,
    int &agent_id)
{

   int command_length_per_agent = 10; // Each agent's command contains 10 elements (vehicle + manipulator)

    // Calculate the starting index for the current agent's data in the uvms_commands->input.data array
    int start_index = agent_id * command_length_per_agent;
    int end_index = start_index + 10;  

    uvms_world[agent_id].force_input.assign(uvms_commands->input.data.begin() + start_index, uvms_commands->input.data.begin() + end_index);

    return controller_interface::return_type::OK;
};

void casadi_uvms::Dynamics::coupled_simulate(int &agent_id) {

};

void casadi_uvms::Dynamics::decoupled_simulate(int &agent_id)
{
    std::vector<casadi::DM> arm_position_(uvms_world[agent_id].current_position.end() - 4,
                                          uvms_world[agent_id].current_position.end());

    std::vector<casadi::DM> arm_velocity_(uvms_world[agent_id].current_velocity.end() - 4,
                                          uvms_world[agent_id].current_velocity.end());

    std::vector<casadi::DM> arm_forces_(uvms_world[agent_id].force_input.end() - 4,
                                        uvms_world[agent_id].force_input.end());

    arm_simulate_argument = {arm_position_,
                             arm_velocity_,
                             arm_forces_,
                             uvms_world[agent_id].model_p,
                             dt};

    // Original code: constructing the vector with 7 elements
    std::vector<casadi::DM> vehicle_pose_(uvms_world[agent_id].current_position.begin(),
                                          uvms_world[agent_id].current_position.begin() + 7);

    std::vector<casadi::DM> vehicle_vel_(uvms_world[agent_id].current_velocity.begin(),
                                         uvms_world[agent_id].current_velocity.begin() + 6);

    std::vector<casadi::DM> vehicle_state;
    vehicle_state.reserve(13);
    vehicle_state.insert(vehicle_state.end(), vehicle_pose_.begin(), vehicle_pose_.end());
    vehicle_state.insert(vehicle_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());

    std::vector<casadi::DM> vehicle_forces_(uvms_world[agent_id].force_input.begin(),
                                            uvms_world[agent_id].force_input.begin() + 6);

    vehicle_simulate_argument = {vehicle_state, vehicle_forces_, dt, uvms_world[agent_id].flow_velocity};

    arm_sim = fun_service.decoupled_manipulator_uvms_dynamics(arm_simulate_argument);
    vehicle_sim = fun_service.decoupled_vehicle_uvms_dynamics(vehicle_simulate_argument);

    uvms_world[agent_id].next_position[0] = vehicle_sim.at(0).nonzeros()[0];
    uvms_world[agent_id].next_position[1] = vehicle_sim.at(0).nonzeros()[1];
    uvms_world[agent_id].next_position[2] = vehicle_sim.at(0).nonzeros()[2];
    uvms_world[agent_id].next_position[3] = vehicle_sim.at(0).nonzeros()[3];
    uvms_world[agent_id].next_position[4] = vehicle_sim.at(0).nonzeros()[4];
    uvms_world[agent_id].next_position[5] = vehicle_sim.at(0).nonzeros()[5];
    uvms_world[agent_id].next_position[6] = vehicle_sim.at(0).nonzeros()[6];

    uvms_world[agent_id].next_velocity[0] = vehicle_sim.at(0).nonzeros()[7];
    uvms_world[agent_id].next_velocity[1] = vehicle_sim.at(0).nonzeros()[8];
    uvms_world[agent_id].next_velocity[2] = vehicle_sim.at(0).nonzeros()[9];
    uvms_world[agent_id].next_velocity[3] = vehicle_sim.at(0).nonzeros()[10];
    uvms_world[agent_id].next_velocity[4] = vehicle_sim.at(0).nonzeros()[11];
    uvms_world[agent_id].next_velocity[5] = vehicle_sim.at(0).nonzeros()[12];

    uvms_world[agent_id].next_position[7] = arm_sim.at(0).nonzeros()[0];
    uvms_world[agent_id].next_position[8] = arm_sim.at(0).nonzeros()[1];
    uvms_world[agent_id].next_position[9] = arm_sim.at(0).nonzeros()[2];
    uvms_world[agent_id].next_position[10] = arm_sim.at(0).nonzeros()[3];

    uvms_world[agent_id].next_velocity[6] = arm_sim.at(0).nonzeros()[4];
    uvms_world[agent_id].next_velocity[7] = arm_sim.at(0).nonzeros()[5];
    uvms_world[agent_id].next_velocity[8] = arm_sim.at(0).nonzeros()[6];
    uvms_world[agent_id].next_velocity[9] = arm_sim.at(0).nonzeros()[7];

    uvms_world[agent_id].force_input[6] = arm_sim.at(1).nonzeros()[0];
    uvms_world[agent_id].force_input[7] = arm_sim.at(1).nonzeros()[1];
    uvms_world[agent_id].force_input[8] = arm_sim.at(1).nonzeros()[2];
    uvms_world[agent_id].force_input[9] = arm_sim.at(1).nonzeros()[3];
};
