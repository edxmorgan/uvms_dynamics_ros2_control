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
#include <sstream> // For std::ostringstream
void casadi_uvms::Dynamics::init_dynamics()
{
    // Use CasADi's "external" to load the compiled dynamics functions
    fun_service.usage_cplusplus_checks("test", "libtest.so", "UVMS Controller");

    fun_service.uvms_dynamics = fun_service.load_casadi_fun("UVMSnext_use_coupled", "libUVMS_xnext.so");

    // fun_service.q2euler = fun_service.load_casadi_fun("q2euler", "libq2eulerf.so");
    // fun_service.euler2q = fun_service.load_casadi_fun("euler2q", "libeuler2qf.so");
    fun_service.pid_controller = fun_service.load_casadi_fun("pid", "libPID.so");
    fun_service.forward_kinematics = fun_service.load_casadi_fun("fkeval", "libFK.so");
    fun_service.uv_G = fun_service.load_casadi_fun("G_n", "libg.so");
    fun_service.uv_J_ned = fun_service.load_casadi_fun("J_", "libJk.so");

    fun_service.uvms_H = fun_service.load_casadi_fun("UVMS_H_use_coupled", "libUVMS_H.so");
    fun_service.uvms_B = fun_service.load_casadi_fun("UVMS_B_use_coupled", "libUVMS_B.so");
    fun_service.uvms_J_ned = fun_service.load_casadi_fun("J_uvms", "libJ_uvms.so");
    fun_service.uvms_optimal_controller = fun_service.load_casadi_fun("opt_cont", "libOptController.so");

    is_coupled = 0;

    manipulator_parameters = {2253.54, 2253.54, 2253.54, 340.4,
                              1e-06, 1e-06, 1e-06, 1e-06,
                              0, 0, 0, 0,
                              2.5, 2.6, 1.7, 0.2,
                              0, 0, 0, 0,
                              4.0, 1.9, 1.3, 1.0};

    vehicle_parameters = {1.15000e+01, 1.12815e+02, 1.14800e+02, 0.00000e+00,
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

    base_To = {3.142, 0.0, 0.0, 0.14, 0.0, -0.12};

    joint_min = {-1000, -1000, -1000, -1000, -1000, -1000,  1, 0.01, 0.01, 0.01};
    joint_max = {1000, 1000, 1000, 1000, 1000, 1000, 1000,  3.4, 3.4, 5.7};
};

std::pair<std::vector<DM>, DM> casadi_uvms::Dynamics::publish_forward_kinematics(
    const rclcpp::Logger & /*logger*/,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/,
    int &agent_id)
{
    DM base_T = DM::vertcat({DM(3.142), DM(0.000), DM(0.000), DM(0.140), DM(0.000), DM(-0.120)});

    double x = uvms_world[agent_id].current_position[0];
    double y = uvms_world[agent_id].current_position[1];
    double z = uvms_world[agent_id].current_position[2];


    q_orig_base.setRPY(uvms_world[agent_id].current_position[3],
                       uvms_world[agent_id].current_position[4],
                       uvms_world[agent_id].current_position[5]);
    q_orig_base.normalize();

    DM state_position = DM::vertcat({DM(x),
                                     DM(y),
                                     DM(z),
                                     DM(q_orig_base.getW()),
                                     DM(q_orig_base.getX()),
                                     DM(q_orig_base.getY()),
                                     DM(q_orig_base.getZ())});

    DM generalized_coordinates = DM::vertcat({0.0,
                                              0.0,
                                              0.0,
                                              0.0,
                                              0.0,
                                              0.0,
                                              DM(uvms_world[agent_id].current_position[6]),
                                              DM(uvms_world[agent_id].current_position[7]),
                                              DM(uvms_world[agent_id].current_position[8]),
                                              DM(uvms_world[agent_id].current_position[9])});

    std::vector<DM> fk_argumt = {generalized_coordinates, base_T};
    std::vector<DM> T_i = fun_service.forward_kinematics(fk_argumt);

    return {T_i, state_position};
};

controller_interface::return_type casadi_uvms::Dynamics::force_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger & /*logger*/,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/,
    int &agent_id)
{
    int command_length_per_agent = static_cast<int>(uvms_world[agent_id].effortCommander.size()); // Each agent's command contains 11 elements (vehicle + manipulator)

    // Calculate the starting index for the current agent's data in the uvms_commands
    int start_index = agent_id * command_length_per_agent;
    int end_index = start_index + command_length_per_agent;

    uvms_world[agent_id].force_input.assign(uvms_commands->force.data.begin() + start_index, uvms_commands->force.data.begin() + end_index);

    return controller_interface::return_type::OK;
};

controller_interface::return_type casadi_uvms::Dynamics::pid_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger & /*logger*/,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/,
    int &agent_id)
{
    std::vector<casadi::DM> arm_position_(uvms_world[agent_id].current_position.end() - 5,
                                          uvms_world[agent_id].current_position.end());

    std::vector<casadi::DM> arm_velocity_(uvms_world[agent_id].current_velocity.end() - 5,
                                          uvms_world[agent_id].current_velocity.end());

    std::vector<casadi::DM> vehicle_pose_(uvms_world[agent_id].current_position.begin(),
                                          uvms_world[agent_id].current_position.begin() + 6);

    std::vector<casadi::DM> vehicle_vel_(uvms_world[agent_id].current_velocity.begin(),
                                         uvms_world[agent_id].current_velocity.begin() + 6);

    std::vector<casadi::DM> uvms_position_state;
    uvms_position_state.reserve(10);
    uvms_position_state.insert(uvms_position_state.end(), vehicle_pose_.begin(), vehicle_pose_.end());
    uvms_position_state.insert(uvms_position_state.end(), arm_position_.begin(), arm_position_.end() - 1);

    std::vector<casadi::DM> uvms_velocity_state;
    uvms_velocity_state.reserve(10);
    uvms_velocity_state.insert(uvms_velocity_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());
    uvms_velocity_state.insert(uvms_velocity_state.end(), arm_velocity_.begin(), arm_velocity_.end() - 1);

    uvms_world[agent_id].Kp = {1, 1, 1, 1, 1, 1, 2.0, 2.0, 2.0, 1.0};
    uvms_world[agent_id].Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.01, 0.01};
    uvms_world[agent_id].Kd = {4, 3, 3, 3, 3, 3, 0.1, 0.1, 0.1, 0.1};

    uvms_world[agent_id].u_min = {-1, -1, -3, -1, -0.1, -1, -1.0, -0.1, -0.1, -0.1};
    uvms_world[agent_id].u_max = {1, 1, 3, 1, 1, 0.1, 1.0, 0.1, 0.1, 0.1};

    int command_length_per_agent = static_cast<int>(uvms_world[agent_id].effortCommander.size()); // Each agent's command contains 11 elements (vehicle + manipulator)

    // Calculate the starting index for the current agent's data in the uvms_commands
    int start_index = agent_id * command_length_per_agent;
    int end_index = start_index + command_length_per_agent;

    // Assign the 10 elements (6 vehicle reference + 4 joints reference) to uvms_world[agent_id].XF
    uvms_world[agent_id].XF.assign(uvms_commands->pose.data.begin() + start_index, uvms_commands->pose.data.begin() + end_index - 1);

    std::vector<casadi::DM> blue_rg = {0.0, 0.0, 0.02};
    std::vector<casadi::DM> blue_rb = {0.0, 0.0, 0.0};

    depth = casadi::DM(uvms_world[agent_id].current_position[2]);
    uv_G_argument = {depth, casadi::DM({uvms_world[agent_id].current_position[3], uvms_world[agent_id].current_position[4], uvms_world[agent_id].current_position[5]}),
                     112.81500000000001, 114.8, blue_rg, blue_rb};
    uv_g = fun_service.uv_G(uv_G_argument);

    uv_J_ned_argument = {casadi::DM({uvms_world[agent_id].current_position[3],
                                     uvms_world[agent_id].current_position[4],
                                     uvms_world[agent_id].current_position[5]})};
    uv_J_ned = fun_service.uv_J_ned(uv_J_ned_argument);

    pid_argument = {uvms_position_state,
                    uvms_velocity_state,
                    uvms_world[agent_id].XF,
                    uv_g.at(0),
                    uv_J_ned.at(0),
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

controller_interface::return_type casadi_uvms::Dynamics::optimal_controller(
    std::shared_ptr<CmdType> &uvms_commands,
    const rclcpp::Logger & /*logger*/,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    const rclcpp::Time &time,
    const rclcpp::Duration & /*period*/,
    int &agent_id)
{
    std::vector<casadi::DM> arm_position_(uvms_world[agent_id].current_position.end() - 5,
                                          uvms_world[agent_id].current_position.end());

    std::vector<casadi::DM> arm_velocity_(uvms_world[agent_id].current_velocity.end() - 5,
                                          uvms_world[agent_id].current_velocity.end());

    std::vector<casadi::DM> vehicle_pose_(uvms_world[agent_id].current_position.begin(),
                                          uvms_world[agent_id].current_position.begin() + 6);

    std::vector<casadi::DM> vehicle_vel_(uvms_world[agent_id].current_velocity.begin(),
                                         uvms_world[agent_id].current_velocity.begin() + 6);

    std::vector<casadi::DM> uvms_state;
    uvms_state.reserve(20);
    uvms_state.insert(uvms_state.end(), vehicle_pose_.begin(), vehicle_pose_.end());
    uvms_state.insert(uvms_state.end(), arm_position_.begin(), arm_position_.end() - 1);
    uvms_state.insert(uvms_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());
    uvms_state.insert(uvms_state.end(), arm_velocity_.begin(), arm_velocity_.end() - 1);

    uvms_world[agent_id].u_min = {-1, -1, -3, -1, -0.1, -1, -1.0, -0.1, -0.1, -0.1};
    uvms_world[agent_id].u_max = {1, 1, 3, 1, 1, 0.1, 1.0, 0.1, 0.1, 0.1};

    int command_length_per_agent = static_cast<int>(uvms_world[agent_id].effortCommander.size()); // Each agent's command contains 11 elements (vehicle + manipulator)

    // Calculate the starting index for the current agent's data in the uvms_commands
    int start_index = agent_id * command_length_per_agent;
    int end_index = start_index + command_length_per_agent;

    // Assign the 10 elements (6 vehicle reference + 4 joints reference) to uvms_world[agent_id].XF
    uvms_world[agent_id].XF.assign(uvms_commands->pose.data.begin() + start_index, uvms_commands->pose.data.begin() + end_index - 1);
    uvms_world[agent_id].VF.assign(uvms_commands->twist.data.begin() + start_index, uvms_commands->twist.data.begin() + end_index - 1);
    uvms_world[agent_id].AF.assign(uvms_commands->acceleration.data.begin() + start_index, uvms_commands->acceleration.data.begin() + end_index - 1);

    std::vector<double> ref_eul_states_kin = {
        uvms_world[agent_id].XF[3], // Roll
        uvms_world[agent_id].XF[4], // Pitch
        uvms_world[agent_id].XF[5]  // Yaw
    };

    dynamics_argument = {is_coupled, uvms_state, vehicle_parameters, manipulator_parameters, base_To};

    uvms_H_ = fun_service.uvms_H(dynamics_argument);
    uvms_B_ = fun_service.uvms_B(dynamics_argument);

    inv_uvms_H_ = casadi::DM::solve(uvms_H_.at(0), casadi::DM::eye(uvms_H_.at(0).size1()));

    uvms_J_ned_argument = {casadi::DM({uvms_world[agent_id].current_position[3],
                                       uvms_world[agent_id].current_position[4],
                                       uvms_world[agent_id].current_position[5]})};

    uvms_J_ned = fun_service.uvms_J_ned(uvms_J_ned_argument);

    uvms_J_REF_ned_argument = {casadi::DM({uvms_world[agent_id].current_position[3],
                                           uvms_world[agent_id].current_position[4],
                                           uvms_world[agent_id].current_position[5]})};
    uvms_J_REF_ned = fun_service.uvms_J_ned(uvms_J_REF_ned_argument);

    // Define a1_v and a2_v as vectors of casadi::DM
    std::vector<casadi::DM> a1_v = {100, 100, 100, 200, 200, 100, 200, 200, 200, 200};
    std::vector<casadi::DM> a2_v = {0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.000001};

    // Initialize a12_v and ac_v with the same size as a1_v and a2_v
    std::vector<casadi::DM> a12_v(a1_v.size());
    std::vector<casadi::DM> ac_v(a1_v.size());

    // Perform element-wise sqrt and subtraction using casadi::sqrt
    for (size_t i = 0; i < a1_v.size(); ++i)
    {
        a12_v[i] = casadi::DM::sqrt(a1_v[i] * a2_v[i]) - 1e-8;
    }

    // Perform element-wise division and multiply by 2
    for (size_t i = 0; i < a12_v.size(); ++i)
    {
        ac_v[i] = 2 * (a12_v[i] / a2_v[i]);
    }

    // Define epsilon_t correctly
    double epsilon_t = 0.0;

    // Prepare optimal_control_params using vertcat as a free function
    // Flatten the vectors into a single vector of casadi::DM
    std::vector<casadi::DM> vertcat_args;
    vertcat_args.insert(vertcat_args.end(), a1_v.begin(), a1_v.end());
    vertcat_args.insert(vertcat_args.end(), a2_v.begin(), a2_v.end());
    vertcat_args.insert(vertcat_args.end(), a12_v.begin(), a12_v.end());
    vertcat_args.insert(vertcat_args.end(), ac_v.begin(), ac_v.end());
    vertcat_args.emplace_back(epsilon_t); // Convert epsilon_t to casadi::DM implicitly

    optimal_control_params = casadi::DM::vertcat(vertcat_args);
    time_seconds = time.seconds();
    // // Convert optimal_control_params to string using the helper function
    // std::string params_str = dm_to_string(optimal_control_params);

    // // Log the optimal_control_params correctly
    // RCLCPP_INFO(
    //     logger,
    //     "Got optimal parameters: %s",
    //     params_str.c_str());

    opt_control_argument = {
        uvms_state,
        optimal_control_params,
        inv_uvms_H_,
        uvms_H_.at(0),
        uvms_B_.at(0),
        uvms_world[agent_id].XF, // q_ref
        uvms_world[agent_id].VF, // dq_ref
        uvms_world[agent_id].AF, // ddq_ref
        uvms_J_ned.at(0),
        uvms_J_REF_ned.at(0),
        time_seconds,
        uvms_world[agent_id].u_min,
        uvms_world[agent_id].u_max};

    optimal_command = fun_service.uvms_optimal_controller(opt_control_argument);

    std::vector<double> optimal_commands = optimal_command.at(0).nonzeros();
    uvms_world[agent_id].lyapunov_energy = optimal_command.at(1).nonzeros()[0];

    std::copy(optimal_commands.begin(), optimal_commands.end(), uvms_world[agent_id].force_input.begin());

    return controller_interface::return_type::OK;
};

void casadi_uvms::Dynamics::simulate(
    const rclcpp::Logger & /*logger*/,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/,
    int &agent_id)
{
    std::vector<casadi::DM> arm_position_(uvms_world[agent_id].current_position.end() - 5,
                                          uvms_world[agent_id].current_position.end());

    std::vector<casadi::DM> arm_velocity_(uvms_world[agent_id].current_velocity.end() - 5,
                                          uvms_world[agent_id].current_velocity.end());

    std::vector<casadi::DM> vehicle_pose_(uvms_world[agent_id].current_position.begin(),
                                          uvms_world[agent_id].current_position.begin() + 6);

    std::vector<casadi::DM> vehicle_vel_(uvms_world[agent_id].current_velocity.begin(),
                                         uvms_world[agent_id].current_velocity.begin() + 6);

    std::vector<casadi::DM> uvms_state;
    uvms_state.reserve(20);
    uvms_state.insert(uvms_state.end(), vehicle_pose_.begin(), vehicle_pose_.end());
    uvms_state.insert(uvms_state.end(), arm_position_.begin(), arm_position_.end() - 1);
    uvms_state.insert(uvms_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());
    uvms_state.insert(uvms_state.end(), arm_velocity_.begin(), arm_velocity_.end() - 1);

    std::vector<casadi::DM> uvms_forces_(uvms_world[agent_id].force_input.begin(),
                                         uvms_world[agent_id].force_input.end() - 1);

    uvms_world[agent_id].joint_min = joint_min;
    uvms_world[agent_id].joint_max = joint_max;

    uvms_simulate_argument = {is_coupled, uvms_state, uvms_forces_, dt, vehicle_parameters, manipulator_parameters, base_To,
                              uvms_world[agent_id].joint_min, uvms_world[agent_id].joint_max};

    uvms_sim = fun_service.uvms_dynamics(uvms_simulate_argument);

    next_states = uvms_sim.at(0).nonzeros();

    uvms_world[agent_id].next_position[0] = next_states[0];
    uvms_world[agent_id].next_position[1] = next_states[1];
    uvms_world[agent_id].next_position[2] = next_states[2];

    uvms_world[agent_id].next_position[3] = next_states[3];
    uvms_world[agent_id].next_position[4] = next_states[4];
    uvms_world[agent_id].next_position[5] = next_states[5];

    uvms_world[agent_id].next_position[6] = next_states[6];
    uvms_world[agent_id].next_position[7] = next_states[7];
    uvms_world[agent_id].next_position[8] = next_states[8];
    uvms_world[agent_id].next_position[9] = next_states[9];

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

    uvms_world[agent_id].next_acceleration[0] = uvms_sim.at(2).nonzeros()[0];
    uvms_world[agent_id].next_acceleration[1] = uvms_sim.at(2).nonzeros()[1];
    uvms_world[agent_id].next_acceleration[2] = uvms_sim.at(2).nonzeros()[2];

    uvms_world[agent_id].next_acceleration[3] = uvms_sim.at(2).nonzeros()[3];
    uvms_world[agent_id].next_acceleration[4] = uvms_sim.at(2).nonzeros()[4];
    uvms_world[agent_id].next_acceleration[5] = uvms_sim.at(2).nonzeros()[5];

    uvms_world[agent_id].next_acceleration[6] = uvms_sim.at(2).nonzeros()[6];
    uvms_world[agent_id].next_acceleration[7] = uvms_sim.at(2).nonzeros()[7];
    uvms_world[agent_id].next_acceleration[8] = uvms_sim.at(2).nonzeros()[8];
    uvms_world[agent_id].next_acceleration[9] = uvms_sim.at(2).nonzeros()[9];

    // RCLCPP_INFO(
    //     logger,
    //     "Got velocity commands: %f,%f,%f,%f,  %f,%f,%f,%f,  %f,%f",
    //     uvms_world[agent_id].next_velocity[0],
    //     uvms_world[agent_id].next_velocity[1],
    //     uvms_world[agent_id].next_velocity[2],
    //     uvms_world[agent_id].next_velocity[3],
    //     uvms_world[agent_id].next_velocity[4],
    //     uvms_world[agent_id].next_velocity[5],
    //     uvms_world[agent_id].next_velocity[6],
    //     uvms_world[agent_id].next_velocity[7],
    //     uvms_world[agent_id].next_velocity[8],
    //     uvms_world[agent_id].next_velocity[9]);
};

// Helper function implementation
std::string casadi_uvms::Dynamics::dm_to_string(const casadi::DM &dm) const
{
    std::ostringstream oss;
    oss << "DM(" << dm.size1() << "x" << dm.size2() << "):\n";
    for (int i = 0; i < dm.size1(); ++i)
    {
        for (int j = 0; j < dm.size2(); ++j)
        {
            oss << dm(i, j).scalar() << " ";
        }
        oss << "\n";
    }
    return oss.str();
}