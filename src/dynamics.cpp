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
    // fun_service.vehicle_velocity_pid = fun_service.load_casadi_fun("pidC", "libPd.so");
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

    std::vector<double> Kp = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    std::vector<double> Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<double> Kd = {2, 2, 2, 2, 2, 2};
    std::vector<double> XF;
    XF.assign(uvms_commands->input.data.begin(), uvms_commands->input.data.begin() + 6);
    vehicle_pose_pid_argument = {Kp, Ki, Kd, uvms_world[agent_id].sum_ki_buffer, dt, vehicle_state, XF};
    vehicle_pose_command = fun_service.vehicle_position_pid(vehicle_pose_pid_argument);
    // Retrieve the non-zero elements
    std::vector<double> pid_commands = vehicle_pose_command.at(0).nonzeros();
    uvms_world[agent_id].sum_ki_buffer = vehicle_pose_command.at(1).nonzeros();

    // Convert the vector to a string for logging
    std::stringstream ss_pid;
    ss_pid << "Non-zero elements of pid commands: ";
    for (const auto &elem : pid_commands)
    {
        ss_pid << elem << " ";
    }
    // Log the non-zero elements
    RCLCPP_INFO(logger, "pid command %s", ss_pid.str().c_str());
    std::copy(pid_commands.begin(), pid_commands.end(), uvms_world[agent_id].force_input.begin());
    return controller_interface::return_type::OK;
};

controller_interface::return_type casadi_uvms::Dynamics::velocity_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr &clock,
    int &agent_id)
{

    // uvms_commands->input.data
    //     model_dynamics.vehicle_vel_pid(agent_id);
    // auto computed_command = std::make_shared<CmdType>();
    // computed_command->input.data.resize(10, 0.0);
    // computed_command->command_type = "velocity";
    // uvms_commands = computed_command;
    return controller_interface::return_type::OK;
};

controller_interface::return_type casadi_uvms::Dynamics::force_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr &clock,
    int &agent_id)
{
    // Copy force input from commands
    uvms_world[agent_id].force_input.assign(uvms_commands->input.data.begin(), uvms_commands->input.data.begin() + 10);
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
