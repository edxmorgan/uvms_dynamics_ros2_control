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

    fun_service.coupled_uvms_dynamics = fun_service.load_casadi_fun("UVMSnext", "libUVMS.so");
    fun_service.q2euler = fun_service.load_casadi_fun("q2euler", "libq2eulerf.so");
    fun_service.euler2q = fun_service.load_casadi_fun("euler2q", "libeuler2qf.so");

    fun_service.forward_kinematics = fun_service.load_casadi_fun("fkeval", "libFK.so");

    fun_service.vehicle_position_pid = fun_service.load_casadi_fun("pidC", "libPd.so");
    fun_service.vehicle_velocity_pid = fun_service.load_casadi_fun("vpidC", "libVPd.so");
};

std::vector<DM> casadi_uvms::Dynamics::publish_foward_kinematics(int &agent_id)
{
    DM q = DM::vertcat({DM(uvms_world[agent_id].current_position[7]), DM(uvms_world[agent_id].current_position[8]),
                        DM(uvms_world[agent_id].current_position[9]), DM(uvms_world[agent_id].current_position[10])});

    DM baseT_xyz = DM::vertcat({DM(0.140), DM(0.000), DM(-0.120)});
    DM baseT_rpy = DM::vertcat({DM(3.142), DM(0.000), DM(0.000)});

    std::vector<double> eul_states = convertQuaternionToEuler(uvms_world[agent_id].current_position[3],
                                                              uvms_world[agent_id].current_position[4],
                                                              uvms_world[agent_id].current_position[5],
                                                              uvms_world[agent_id].current_position[6]);

    DM pn = DM::vertcat({DM(uvms_world[agent_id].current_position[0]),
                         DM(uvms_world[agent_id].current_position[1]),
                         DM(uvms_world[agent_id].current_position[2]),
                         DM(eul_states[0]),
                         DM(eul_states[1]),
                         DM(eul_states[2])});
    std::vector<DM> fk_argumt = {q, baseT_xyz, baseT_rpy, pn};
    std::vector<DM> T_i = fun_service.forward_kinematics(fk_argumt);
    return T_i;
};


controller_interface::return_type casadi_uvms::Dynamics::force_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr &clock,
    int &agent_id)
{
    int command_length_per_agent = static_cast<int>(uvms_world[agent_id].effortCommander.size()); // Each agent's command contains 10 elements (vehicle + manipulator)

    // Calculate the starting index for the current agent's data in the uvms_commands->input.data array
    int start_index = agent_id * command_length_per_agent;
    int end_index = start_index + command_length_per_agent;

    uvms_world[agent_id].force_input.assign(uvms_commands->input.data.begin() + start_index, uvms_commands->input.data.begin() + end_index);

    return controller_interface::return_type::OK;
};

void casadi_uvms::Dynamics::coupled_simulate(int &agent_id)
{
    // Log the uvms_simulate_argument
    // std::cout << "uvms_simulate_argument:" <<  agent_id << std::endl;
    // for (const auto &arg : uvms_world[agent_id].current_position)
    // {
    //     std::cout << arg << std::endl;
    // }
    std::vector<casadi::DM> arm_position_(uvms_world[agent_id].current_position.end() - 5,
                                          uvms_world[agent_id].current_position.end());

    std::vector<casadi::DM> arm_velocity_(uvms_world[agent_id].current_velocity.end() - 5,
                                          uvms_world[agent_id].current_velocity.end());

    // std::vector<casadi::DM> arm_forces_(uvms_world[agent_id].force_input.end() - 5,
    //                                     uvms_world[agent_id].force_input.end());

    std::vector<casadi::DM> vehicle_pose_(uvms_world[agent_id].current_position.begin(),
                                          uvms_world[agent_id].current_position.begin() + 7);

    std::vector<casadi::DM> vehicle_vel_(uvms_world[agent_id].current_velocity.begin(),
                                         uvms_world[agent_id].current_velocity.begin() + 6);

    std::vector<double> eul_states = convertQuaternionToEuler(uvms_world[agent_id].current_position[3],
                                                              uvms_world[agent_id].current_position[4],
                                                              uvms_world[agent_id].current_position[5],
                                                              uvms_world[agent_id].current_position[6]);

    std::vector<casadi::DM> uvms_state;
    uvms_state.reserve(20);
    uvms_state.insert(uvms_state.end(), vehicle_pose_.begin(), vehicle_pose_.begin() + 3);
    uvms_state.insert(uvms_state.end(), eul_states.begin(), eul_states.end());

    uvms_state.insert(uvms_state.end(), arm_position_.begin(), arm_position_.end()-1);
    uvms_state.insert(uvms_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());
    uvms_state.insert(uvms_state.end(), arm_velocity_.begin(), arm_velocity_.end()-1);

    std::vector<casadi::DM> uvms_forces_(uvms_world[agent_id].force_input.begin(),
                                         uvms_world[agent_id].force_input.end() -1);

    std::vector<casadi::DM> parameter_values = {2253.54, 2253.54, 2253.54, 340.4, 0, 0, 0, 1e-05, 0, 0, 0, 0, 0, 0, 1e-05,
                                                0, 0, 0, 1e-05, 0, 1e-05, 0, 0, 0, 0, 3, 2.3, 2.2, 0.3, 0, 0, 0, 0, 3, 1.8, 1, 1.15, 0, 0, 0, 0, 0, 0, 0, 7e-06, 7e-06, 0,
                                                0.032, 0.032, 0.017, 0, 0.001716, 0.001716, 0.017, 0.201, 0.201, 7e-06, 0, 7e-06, 0.032, 0.017, 0.032, 0.002443, 0.002443, 0,
                                                0.226, 0.226, 0.017, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.26, 0.26, 0.3, 0, 0,
                                                0, 0.3, 1.6, 1.6, 0, 0, 0, 0.26, 0.3, 0.26, 0, 0, 0, 1.8, 1.8, 0.3, 1.8e-05, 0.000203, 2.5e-05, 0.000155, -0.001, -0.002, -0.032,
                                                0.073, 0, -0.002, 0.003, 0.001, -0.017, 0, 0.003,
                                                -0.098, 1, 0, 0, 0, 0, 0, 0, dt, 3.142, 0, 0, 0.14, 0, -0.12, 0, 1.5, 0.05, 0, 5.7, 3.4, 3.4, 5.7};

    uvms_simulate_argument = {uvms_state,
                              uvms_forces_,
                              parameter_values};

    uvms_sim = fun_service.coupled_uvms_dynamics(uvms_simulate_argument);

    next_states = uvms_sim.at(0).nonzeros();

    std::vector<double> quat_states = convertEulerToQuaternion(next_states[3], next_states[4], next_states[5]);

    uvms_world[agent_id].next_position[0] = next_states[0];
    uvms_world[agent_id].next_position[1] = next_states[1];
    uvms_world[agent_id].next_position[2] = next_states[2];

    uvms_world[agent_id].next_position[3] = quat_states[0];
    uvms_world[agent_id].next_position[4] = quat_states[1];
    uvms_world[agent_id].next_position[5] = quat_states[2];
    uvms_world[agent_id].next_position[6] = quat_states[3];

    uvms_world[agent_id].next_position[7] = next_states[6];
    uvms_world[agent_id].next_position[8] = next_states[7];
    uvms_world[agent_id].next_position[9] = next_states[8];
    uvms_world[agent_id].next_position[10] = next_states[9];

    uvms_world[agent_id].next_velocity[0] = next_states[10];
    uvms_world[agent_id].next_velocity[1] = next_states[11];
    uvms_world[agent_id].next_velocity[2] = next_states[12];
    uvms_world[agent_id].next_velocity[3] = next_states[13];
    uvms_world[agent_id].next_velocity[4] = next_states[14];
    uvms_world[agent_id].next_velocity[5] = next_states[15];

    uvms_world[agent_id].next_velocity[6] = next_states[16];
    uvms_world[agent_id].next_velocity[7] = next_states[17];
    uvms_world[agent_id].next_velocity[8] = next_states[18];
    uvms_world[agent_id].next_velocity[9] = next_states[19];

    // uvms_world[agent_id].force_input[6] = arm_sim.at(1).nonzeros()[0];
    // uvms_world[agent_id].force_input[7] = arm_sim.at(1).nonzeros()[1];
    // uvms_world[agent_id].force_input[8] = arm_sim.at(1).nonzeros()[2];
    // uvms_world[agent_id].force_input[9] = arm_sim.at(1).nonzeros()[3];
};

std::vector<double> casadi_uvms::Dynamics::convertEulerToQuaternion(const double r, const double p, const double y)
{
    casadi::DM euler_vector = casadi::DM::vertcat({r, p, y});
    // Convert euler to quaternion states
    quaternion_states = fun_service.euler2q(euler_vector);

    return quaternion_states.at(0).nonzeros();
};

std::vector<double> casadi_uvms::Dynamics::convertQuaternionToEuler(const double w, const double x, const double y, const double z)
{
    casadi::DM quaternion_vector = casadi::DM::vertcat({w, x, y, z});
    // Convert quaternion to euler states
    euler_states = fun_service.q2euler(quaternion_vector);
    return euler_states.at(0).nonzeros();
};