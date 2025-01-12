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

    fun_service.uvms_dynamics = fun_service.load_casadi_fun("UVMSnext_use_coupled", "libUVMS_xnext.so");

    fun_service.q2euler = fun_service.load_casadi_fun("q2euler", "libq2eulerf.so");
    fun_service.euler2q = fun_service.load_casadi_fun("euler2q", "libeuler2qf.so");
    fun_service.pid_controller = fun_service.load_casadi_fun("pid", "libPID.so");
    fun_service.forward_kinematics = fun_service.load_casadi_fun("fkeval", "libFK.so");
    fun_service.uv_G = fun_service.load_casadi_fun("G_n", "libg.so");
    fun_service.uv_J = fun_service.load_casadi_fun("J_", "libJk.so");
};

std::pair<std::vector<DM>, DM> casadi_uvms::Dynamics::publish_forward_kinematics(
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr &clock,
    int &agent_id)
{
    DM base_T = DM::vertcat({DM(3.142), DM(0.000), DM(0.000), DM(0.140), DM(0.000), DM(-0.120)});

    q_orig_base.setW(uvms_world[agent_id].current_position[3]);
    q_orig_base.setX(uvms_world[agent_id].current_position[4]);
    q_orig_base.setY(uvms_world[agent_id].current_position[5]);
    q_orig_base.setZ(uvms_world[agent_id].current_position[6]);

    // Rotate the pose about X UPRIGHT
    q_rot_base.setRPY(M_PI, 0.0, 0.0);
    q_new_base = q_orig_base * q_rot_base;
    q_new_base.normalize();

    tf2::Matrix3x3(q_new_base).getRPY(roll, pitch, yaw);

    double x = uvms_world[agent_id].current_position[0];
    double y = -uvms_world[agent_id].current_position[1];
    double z = uvms_world[agent_id].current_position[2];
    DM generalized_coordinates = DM::vertcat({DM(x),
                                              DM(y),
                                              DM(z),
                                              DM(roll),
                                              DM(pitch),
                                              DM(yaw),
                                              DM(uvms_world[agent_id].current_position[7]),
                                              DM(uvms_world[agent_id].current_position[8]),
                                              DM(uvms_world[agent_id].current_position[9]),
                                              DM(uvms_world[agent_id].current_position[10])});

    DM qned = DM::vertcat({DM(x),
                           DM(y),
                           DM(z),
                           DM(q_new_base.w()),
                           DM(q_new_base.x()),
                           DM(q_new_base.y()),
                           DM(q_new_base.z())});

    std::vector<DM> fk_argumt = {generalized_coordinates, base_T};
    std::vector<DM> T_i = fun_service.forward_kinematics(fk_argumt);

    return {T_i, qned};
};

controller_interface::return_type casadi_uvms::Dynamics::pid_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr &clock,
    int &agent_id)
{
    std::vector<casadi::DM> arm_position_(uvms_world[agent_id].current_position.end() - 5,
                                          uvms_world[agent_id].current_position.end());

    std::vector<casadi::DM> arm_velocity_(uvms_world[agent_id].current_velocity.end() - 5,
                                          uvms_world[agent_id].current_velocity.end());

    std::vector<casadi::DM> vehicle_pose_(uvms_world[agent_id].current_position.begin(),
                                          uvms_world[agent_id].current_position.begin() + 7);

    std::vector<casadi::DM> vehicle_vel_(uvms_world[agent_id].current_velocity.begin(),
                                         uvms_world[agent_id].current_velocity.begin() + 6);

    std::vector<double> eul_states = convertQuaternionToEuler(uvms_world[agent_id].current_position[3],
                                                              uvms_world[agent_id].current_position[4],
                                                              uvms_world[agent_id].current_position[5],
                                                              uvms_world[agent_id].current_position[6]);

    std::vector<casadi::DM> uvms_position_state;
    uvms_position_state.reserve(10);
    uvms_position_state.insert(uvms_position_state.end(), vehicle_pose_.begin(), vehicle_pose_.begin() + 3);
    uvms_position_state.insert(uvms_position_state.end(), eul_states.begin(), eul_states.end());
    uvms_position_state.insert(uvms_position_state.end(), arm_position_.begin(), arm_position_.end() - 1);

    std::vector<casadi::DM> uvms_velocity_state;
    uvms_velocity_state.reserve(10);
    uvms_velocity_state.insert(uvms_velocity_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());
    uvms_velocity_state.insert(uvms_velocity_state.end(), arm_velocity_.begin(), arm_velocity_.end() - 1);

    // uvms_world[agent_id].Kp = {1, 1, 1, 1, 1, 1, 2.0, 2.0, 2.0, 1.0};
    // uvms_world[agent_id].Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.01, 0.01};
    // uvms_world[agent_id].Kd = {4, 3, 3, 3, 3, 3, 0.1, 0.1, 0.1, 0.1};

    // <param name="min_position">1.0</param>  <param name="max_position">5.5</param>
    // <param name="min_position">0.01</param>  <param name="max_position">3.400</param>
    // <param name="min_position">0.01</param>  <param name="max_position">3.400</param>
    // <param name="min_position">0.01</param> <param name="max_position">5.70</param>

    // provided gains --> 0
    // uvms_world[agent_id].Kp = {1, 1, 1, 1, 1, 1, 7.8516, 6.2398, 2.3119, 1.8381};
    // uvms_world[agent_id].Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000};
    // uvms_world[agent_id].Kd = {4, 3, 3, 3, 3, 3, 3.6438, 4.8058, 2.8552, 6.2847};

    // provided gains --> 1
    // uvms_world[agent_id].Kp = {1, 1, 1, 1, 1, 1, 7.9175, 6.5515, 2.2020, 1.6566};
    // uvms_world[agent_id].Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000};
    // uvms_world[agent_id].Kd = {4, 3, 3, 3, 3, 3, 3.6910, 5.0202, 2.7693, 5.2407};

    // provided gains --> 2
    uvms_world[agent_id].Kp = {1, 1, 1, 1, 1, 1, 7.9011, 6.4971, 2.1482, 1.6117};
    uvms_world[agent_id].Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0000, 0.0000, 0.0000, 0.0000};
    uvms_world[agent_id].Kd = {4, 3, 3, 3, 3, 3, 3.5290, 5.0801, 2.7392, 5.8232};

    uvms_world[agent_id].u_min = {-10, -10, -10, -10, -10, -10, -2.83664, -0.629139, -0.518764, -0.54};
    uvms_world[agent_id].u_max = {10, 10, 10, 10, 10, 10, 2.83664, 0.629139, 0.518764, 0.54};

    int command_length_per_agent = 10; // Each agent's command contains 10 elements (position + quaternion + velocity)

    // Calculate the starting index for the current agent's data in the uvms_commands->input.data array
    int start_index = agent_id * command_length_per_agent;
    int end_index = start_index + 10; // We are only interested in the first 10 elements (6 vehicle reference + 4 joints reference)

    // Assign the 10 elements (6 vehicle reference + 4 joints reference) to uvms_world[agent_id].XF
    uvms_world[agent_id].XF.assign(uvms_commands->input.data.begin() + start_index, uvms_commands->input.data.begin() + end_index);

    std::vector<casadi::DM> blue_rg = {0., 0., 0.02};
    std::vector<casadi::DM> blue_rb = {0, 0, 0};
    std::vector<double> eul_states_kin = convertQuaternionToEuler(uvms_world[agent_id].current_position[3],
                                                                  uvms_world[agent_id].current_position[4],
                                                                  uvms_world[agent_id].current_position[5],
                                                                  uvms_world[agent_id].current_position[6]);

    depth = casadi::DM(uvms_world[agent_id].current_position[2]);
    uv_G_argument = {depth, casadi::DM(eul_states_kin), 112.81500000000001, 114.8, blue_rg, blue_rb};
    uv_g = fun_service.uv_G(uv_G_argument);

    uv_J_argument = {casadi::DM(eul_states_kin)};
    uv_J = fun_service.uv_J(uv_J_argument);

    pid_argument = {uvms_position_state,
                    uvms_velocity_state,
                    uvms_world[agent_id].XF,
                    uv_g.at(0),
                    uv_J.at(0),
                    uvms_world[agent_id].Kp,
                    uvms_world[agent_id].Ki,
                    uvms_world[agent_id].Kd,
                    uvms_world[agent_id].sum_ki_buffer,
                    dt,
                    uvms_world[agent_id].u_min, uvms_world[agent_id].u_max};
    pid_command = fun_service.pid_controller(pid_argument);

    std::vector<double> pid_commands = pid_command.at(0).nonzeros();
    uvms_world[agent_id].sum_ki_buffer = pid_command.at(1).nonzeros();

    std::copy(pid_commands.begin(), pid_commands.end(), uvms_world[agent_id].force_input.begin());

    return controller_interface::return_type::OK;
};

controller_interface::return_type casadi_uvms::Dynamics::force_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger & /*logger*/,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    int &agent_id)
{
    int command_length_per_agent = static_cast<int>(uvms_world[agent_id].effortCommander.size()); // Each agent's command contains 10 elements (vehicle + manipulator)

    // Calculate the starting index for the current agent's data in the uvms_commands->input.data array
    int start_index = agent_id * command_length_per_agent;
    int end_index = start_index + command_length_per_agent;

    uvms_world[agent_id].force_input.assign(uvms_commands->input.data.begin() + start_index, uvms_commands->input.data.begin() + end_index);

    return controller_interface::return_type::OK;
};

void casadi_uvms::Dynamics::simulate(
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr &clock,
    int &agent_id)
{
    std::vector<casadi::DM> arm_position_(uvms_world[agent_id].current_position.end() - 5,
                                          uvms_world[agent_id].current_position.end());

    std::vector<casadi::DM> arm_velocity_(uvms_world[agent_id].current_velocity.end() - 5,
                                          uvms_world[agent_id].current_velocity.end());

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

    uvms_state.insert(uvms_state.end(), arm_position_.begin(), arm_position_.end() - 1);
    uvms_state.insert(uvms_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());
    uvms_state.insert(uvms_state.end(), arm_velocity_.begin(), arm_velocity_.end() - 1);

    std::vector<casadi::DM> uvms_forces_(uvms_world[agent_id].force_input.begin(),
                                         uvms_world[agent_id].force_input.end() - 1);

    std::vector<casadi::DM> manipulator_parameters = {2253.54, 2253.54, 2253.54, 340.4, 1e-06, 1e-06, 1e-06, 1e-06,
                                                      0, 0, 0, 0,
                                                      3, 2.3, 2.2, 0.3,
                                                      0, 0, 0, 0,
                                                      3, 1.8, 1, 1.15};

    std::vector<casadi::DM> vehicle_parameters = {1.15000e+01, 1.12815e+02, 1.14800e+02, 0.00000e+00,
                                                  0.00000e+00, 2.00000e-02, 0.00000e+00, 0.00000e+00,
                                                  0.00000e+00, 1.60000e-01, 1.60000e-01, 1.60000e-01,
                                                  0.00000e+00, -5.50000e+00, -1.27000e+01, -1.45700e+01,
                                                  -1.20000e-01, -1.20000e-01, -1.20000e-01, 0.00000e+00,
                                                  0.00000e+00, 0.00000e+00, 0.00000e+00, -4.03000e+00,
                                                  -6.22000e+00, -5.18000e+00, -7.00000e-02, -7.00000e-02,
                                                  -7.00000e-02, -1.81800e+01, -2.16600e+01, -3.69900e+01,
                                                  -1.55000e+00, -1.55000e+00, -1.55000e+00, 0.00000e+00,
                                                  0.00000e+00, 0.00000e+00, 0.00000e+00, 0.00000e+00,
                                                  0.00000e+00};

    std::vector<casadi::DM> base_To = {3.142, 0.0, 0.0, 0.14, 0.0, -0.12};

    std::vector<casadi::DM> joint_min = {0.00, 1.50, 0.10, 0.10};

    std::vector<casadi::DM> joint_max = {5.50, 3.40, 3.40, 5.70};

    casadi::DM is_coupled = 1;

    uvms_simulate_argument = {is_coupled, uvms_state, uvms_forces_, dt, manipulator_parameters, vehicle_parameters, base_To, joint_min, joint_max};

    uvms_sim = fun_service.uvms_dynamics(uvms_simulate_argument);

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

    uvms_world[agent_id].force_input[0] = uvms_sim.at(1).nonzeros()[0];
    uvms_world[agent_id].force_input[1] = uvms_sim.at(1).nonzeros()[1];
    uvms_world[agent_id].force_input[2] = uvms_sim.at(1).nonzeros()[2];
    uvms_world[agent_id].force_input[3] = uvms_sim.at(1).nonzeros()[3];
    uvms_world[agent_id].force_input[4] = uvms_sim.at(1).nonzeros()[4];
    uvms_world[agent_id].force_input[5] = uvms_sim.at(1).nonzeros()[5];

    uvms_world[agent_id].force_input[6] = uvms_sim.at(1).nonzeros()[6];
    uvms_world[agent_id].force_input[7] = uvms_sim.at(1).nonzeros()[7];
    uvms_world[agent_id].force_input[8] = uvms_sim.at(1).nonzeros()[8];
    uvms_world[agent_id].force_input[9] = uvms_sim.at(1).nonzeros()[9];

    //     RCLCPP_INFO(
    //         logger,
    //         "Got commands: %f,%f,%f,%f,  %f,%f,%f,%f,  %f,%f",
    //         uvms_world[agent_id].force_input[0],
    //         uvms_world[agent_id].force_input[1],
    //         uvms_world[agent_id].force_input[2],
    //         uvms_world[agent_id].force_input[3],
    //         uvms_world[agent_id].force_input[4],
    //         uvms_world[agent_id].force_input[5],
    //         uvms_world[agent_id].force_input[6],
    //         uvms_world[agent_id].force_input[7],
    //         uvms_world[agent_id].force_input[8],
    //         uvms_world[agent_id].force_input[9]);
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