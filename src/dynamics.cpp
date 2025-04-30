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

#include "uvms_controller/dynamics.hpp"
#include <sstream> // For std::ostringstream

#if __has_include("uvms_controller/dynamics_params.hpp")
//   #pragma message("Private parameters enabled (header found)")
#include "uvms_controller/dynamics_params.hpp"
#else
//   #pragma message("Fallback parameters used")

const std::vector<casadi::DM> private_vehicle_parameters = {1.15000000e+01, 1.12815000e+02, 1.14800000e+02, 0.00000000e+00,
                                                            0.00000000e+00, 2.00000000e-02, 0.00000000e+00, 0.00000000e+00,
                                                            0.00000000e+00, 1.60000000e-01, 1.60000000e-01, 1.60000000e-01,
                                                            0.00000000e+00, -5.50000000e+00, -1.27000000e+01, -1.45700000e+01,
                                                            -1.20000000e-01, -1.20000000e-01, -1.20000000e-01, 0.00000000e+00,
                                                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00, -4.03000000e+00,
                                                            -6.22000000e+00, -5.18000000e+00, -7.00000000e-02, -7.00000000e-02,
                                                            -7.00000000e-02, -1.81800000e+01, -2.16600000e+01, -3.69900000e+01,
                                                            -1.55000000e+00, -1.55000000e+00, -1.55000000e+00, 0.00000000e+00,
                                                            1.00421848e+00, 1.00000000e+00, 1.00000000e+00, 1.00000000e+00,
                                                            1.00000000e+00, 1.00000000e+00, 1.00000000e+00, 0.00000000e+00,
                                                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                                                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00, 0.00000000e+00,
                                                            0.00000000e+00, 0.00000000e+00, 0.00000000e+00};

// ── Direction-dependent coefficients  (fwd0 … fwd3, rev0 … rev3) ────────────────
const std::vector<casadi::DM> sgn_qdot_k = {50.0, 50.0, 50.0, 50.0,  50.0, 50.0, 50.0, 50.0};
const std::vector<casadi::DM> viscous   = { 5,5,5,5,   5,5,5,5 };
const std::vector<casadi::DM> coulomb   = { 0,0,0,0,   0,0,0,0 };
const std::vector<casadi::DM> I_Grotor  = { 3,3,3,3,   3,3,3,3 };
const casadi::DM gravity = -9.81;
const std::vector<casadi::DM> joint_min = {-1000, -1000, -1000, -1000, -1000, -1000, 1, 0.01, 0.01, 0.01};
const std::vector<casadi::DM> joint_max = {1000, 1000, 1000, 1000, 1000, 1000, 5.50, 3.40, 3.40, 5.70};
#endif

void casadi_uvms::Dynamics::init_dynamics()
{
    // Use CasADi's "external" to load the compiled dynamics functions
    fun_service.usage_cplusplus_checks("test", "libtest.so", "UVMS Controller");

    fun_service.uv_dynamics = fun_service.load_casadi_fun("Vnext", "libUV_xnext.so");
    fun_service.arm_dynamics = fun_service.load_casadi_fun("Mnext", "libMnext.so");

    fun_service.pid_controller = fun_service.load_casadi_fun("pid", "libPID.so");
    fun_service.forward_kinematics = fun_service.load_casadi_fun("fkeval", "libFK.so");
    fun_service.uv_G = fun_service.load_casadi_fun("G_n", "libg.so");
    fun_service.uv_J_ned = fun_service.load_casadi_fun("J_", "libJk.so");

    fun_service.arm_H = fun_service.load_casadi_fun("M_eval", "libMass_arm.so");
    fun_service.arm_B = fun_service.load_casadi_fun("C_eval", "libBias_arm.so");

    fun_service.vehicle_H = fun_service.load_casadi_fun("M_b", "libMass_vehicle.so");
    fun_service.vehicle_B = fun_service.load_casadi_fun("Bias_b", "libBias_vehicle.so");

    fun_service.arm_optimal_controller = fun_service.load_casadi_fun("opt_cont", "libOptArmController.so");
    fun_service.uv_optimal_controller = fun_service.load_casadi_fun("opt_cont", "libOptUVController.so");

    fun_service.f_base = fun_service.load_casadi_fun("F_base_", "libF_base.so");
    is_coupled = 0;

    manipulator_parameters = {};
    manipulator_parameters.insert(manipulator_parameters.end(), sgn_qdot_k.begin(), sgn_qdot_k.end());
    manipulator_parameters.insert(manipulator_parameters.end(), viscous.begin(), viscous.end());
    manipulator_parameters.insert(manipulator_parameters.end(), coulomb.begin(), coulomb.end());
    manipulator_parameters.insert(manipulator_parameters.end(), I_Grotor.begin(), I_Grotor.end());

    vehicle_parameters = private_vehicle_parameters;
    base_To = {3.142, 0.0, 0.0, 0.19, 0.0, -0.12};
};
// -DUSE_PRIVATE_PARAMS
std::pair<std::vector<DM>, DM> casadi_uvms::Dynamics::publish_forward_kinematics(
    const rclcpp::Logger & /*logger*/,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/,
    int &agent_id)
{
    DM base_T = DM::vertcat({DM(3.142), DM(0.000), DM(0.000), DM(0.190), DM(0.000), DM(-0.120)});

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
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    const rclcpp::Time & /*time*/,
    const rclcpp::Duration & /*period*/,
    int &agent_id)
{

    RCLCPP_DEBUG(
        logger,
        "Got positions to pid: %f,%f,%f, %f,%f,%f, %f,%f,%f,%f",
        static_cast<double>(uvms_world[agent_id].current_position[0]),
        static_cast<double>(uvms_world[agent_id].current_position[1]),
        static_cast<double>(uvms_world[agent_id].current_position[2]),

        static_cast<double>(uvms_world[agent_id].current_position[3]),
        static_cast<double>(uvms_world[agent_id].current_position[4]),
        static_cast<double>(uvms_world[agent_id].current_position[5]),

        static_cast<double>(uvms_world[agent_id].current_position[6]),
        static_cast<double>(uvms_world[agent_id].current_position[7]),
        static_cast<double>(uvms_world[agent_id].current_position[8]),
        static_cast<double>(uvms_world[agent_id].current_position[9]));

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

    uvms_world[agent_id].Kp = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 2.0, 2.0, 2.0, 1.0};
    uvms_world[agent_id].Ki = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.01, 0.01, 0.01};
    uvms_world[agent_id].Kd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.1, 0.1};

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
                     vehicle_parameters[1], vehicle_parameters[2], blue_rg, blue_rb};
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
    const rclcpp::Logger &logger,
    const rclcpp::Clock::SharedPtr & /*clock*/,
    const rclcpp::Time &time,
    const rclcpp::Duration & /*period*/,
    int &agent_id)
{
    // to be fixed
    // // Ensure force_input has 11 elements, all initialized to zero.
    // uvms_world[agent_id].force_input.assign(11, 0);
    // time_seconds = time.seconds();
    // std::vector<casadi::DM> arm_position_(uvms_world[agent_id].current_position.end() - 5,
    //                                       uvms_world[agent_id].current_position.end());

    // std::vector<casadi::DM> arm_velocity_(uvms_world[agent_id].current_velocity.end() - 5,
    //                                       uvms_world[agent_id].current_velocity.end());

    // std::vector<casadi::DM> vehicle_pose_(uvms_world[agent_id].current_position.begin(),
    //                                       uvms_world[agent_id].current_position.begin() + 6);

    // std::vector<casadi::DM> vehicle_vel_(uvms_world[agent_id].current_velocity.begin(),
    //                                      uvms_world[agent_id].current_velocity.begin() + 6);

    // std::vector<casadi::DM> uvms_position_state;
    // uvms_position_state.reserve(10);
    // uvms_position_state.insert(uvms_position_state.end(), vehicle_pose_.begin(), vehicle_pose_.end());
    // uvms_position_state.insert(uvms_position_state.end(), arm_position_.begin(), arm_position_.end() - 1);

    // std::vector<casadi::DM> uvms_velocity_state;
    // uvms_velocity_state.reserve(10);
    // uvms_velocity_state.insert(uvms_velocity_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());
    // uvms_velocity_state.insert(uvms_velocity_state.end(), arm_velocity_.begin(), arm_velocity_.end() - 1);

    // int command_length_per_agent = static_cast<int>(uvms_world[agent_id].effortCommander.size()); // Each agent's command contains 11 elements (vehicle + manipulator)

    // // Calculate the starting index for the current agent's data in the uvms_commands
    // int start_index = agent_id * command_length_per_agent;
    // int end_index = start_index + command_length_per_agent;

    // // Assign the 10 elements (6 vehicle reference + 4 joints reference) to uvms_world[agent_id].XF
    // uvms_world[agent_id].XF.assign(uvms_commands->pose.data.begin() + start_index, uvms_commands->pose.data.begin() + end_index - 1);
    // uvms_world[agent_id].VF.assign(uvms_commands->twist.data.begin() + start_index, uvms_commands->twist.data.begin() + end_index - 1);
    // uvms_world[agent_id].AF.assign(uvms_commands->acceleration.data.begin() + start_index, uvms_commands->acceleration.data.begin() + end_index - 1);

    // std::vector<casadi::DM> manipulator_position_state(uvms_position_state.begin() + 6,
    //                                                    uvms_position_state.end());
    // std::vector<casadi::DM> manipulator_velocity_state(uvms_velocity_state.begin() + 6,
    //                                                    uvms_velocity_state.end());

    // // Build argument containers for arm_H and arm_B.
    // arm_H_argument = {manipulator_position_state,
    //                   manipulator_velocity_state,
    //                   manipulator_parameters,
    //                   gravity};
    // arm_B_argument = {manipulator_position_state,
    //                   manipulator_velocity_state,
    //                   manipulator_parameters,
    //                   gravity};

    // arm_H_ = fun_service.arm_H(arm_H_argument);
    // arm_B_ = fun_service.arm_B(arm_B_argument);
    // inv_arm_H_ = casadi::DM::solve(arm_H_.at(0), casadi::DM::eye(arm_H_.at(0).size1()));

    // arm_J_ned = casadi::DM::eye(4);
    // arm_J_REF_ned = casadi::DM::eye(4);

    // // Define a1_v and a2_v as vectors of casadi::DM
    // std::vector<casadi::DM> arm_a1_v = {100, 100, 100, 100};
    // std::vector<casadi::DM> arm_a2_v = {0.1, 0.1, 0.1, 0.1};

    // // Initialize a12_v and ac_v with the same size as a1_v and a2_v
    // std::vector<casadi::DM> arm_a12_v(arm_a1_v.size());
    // std::vector<casadi::DM> arm_ac_v(arm_a1_v.size());

    // // Perform element-wise sqrt and subtraction using casadi::sqrt
    // for (size_t i = 0; i < arm_a1_v.size(); ++i)
    // {
    //     arm_a12_v[i] = casadi::DM::sqrt(arm_a1_v[i] * arm_a2_v[i]) - 1e-8;
    // }

    // // Perform element-wise division and multiply by 2
    // for (size_t i = 0; i < arm_a12_v.size(); ++i)
    // {
    //     arm_ac_v[i] = 2 * (arm_a12_v[i] / arm_a2_v[i]);
    // }

    // // Define epsilon_t correctly
    // double arm_epsilon_t = 0.0;

    // // Prepare optimal_control_params using vertcat as a free function
    // // Flatten the vectors into a single vector of casadi::DM
    // std::vector<casadi::DM> arm_vertcat_args;
    // arm_vertcat_args.insert(arm_vertcat_args.end(), arm_a1_v.begin(), arm_a1_v.end());
    // arm_vertcat_args.insert(arm_vertcat_args.end(), arm_a2_v.begin(), arm_a2_v.end());
    // arm_vertcat_args.insert(arm_vertcat_args.end(), arm_a12_v.begin(), arm_a12_v.end());
    // arm_vertcat_args.insert(arm_vertcat_args.end(), arm_ac_v.begin(), arm_ac_v.end());
    // arm_vertcat_args.emplace_back(arm_epsilon_t); // Convert arm_epsilon_t to casadi::DM implicitly

    // arm_optimal_control_params = casadi::DM::vertcat(arm_vertcat_args);

    // // Convert optimal_control_params to string using the helper function
    // std::string arm_params_str = dm_to_string(arm_optimal_control_params);

    // // Log the optimal_control_params correctly
    // RCLCPP_DEBUG(
    //     logger,
    //     "Got optimal parameters: %s",
    //     arm_params_str.c_str());

    // std::vector<casadi::DM> manipulator_states;
    // manipulator_states.reserve(8);
    // manipulator_states.insert(manipulator_states.end(), manipulator_position_state.begin(), manipulator_position_state.end());
    // manipulator_states.insert(manipulator_states.end(), manipulator_velocity_state.begin(), manipulator_velocity_state.end());

    // std::vector<casadi::DM> arm_q_ref(uvms_world[agent_id].XF.begin() + 6, uvms_world[agent_id].XF.end());
    // std::vector<casadi::DM> arm_dq_ref(uvms_world[agent_id].VF.begin() + 6, uvms_world[agent_id].VF.end());
    // std::vector<casadi::DM> arm_ddq_ref(uvms_world[agent_id].AF.begin() + 6, uvms_world[agent_id].AF.end());

    // std::vector<casadi::DM> arm_u_min(uvms_world[agent_id].u_min.begin() + 6, uvms_world[agent_id].u_min.end());
    // std::vector<casadi::DM> arm_u_max(uvms_world[agent_id].u_max.begin() + 6, uvms_world[agent_id].u_max.end());

    // arm_opt_control_argument = {
    //     manipulator_states,
    //     arm_optimal_control_params,
    //     inv_arm_H_,
    //     arm_H_.at(0),
    //     arm_B_.at(0),
    //     arm_q_ref,   // q_ref
    //     arm_dq_ref,  // dq_ref
    //     arm_ddq_ref, // ddq_ref
    //     arm_J_ned,
    //     arm_J_REF_ned,
    //     time_seconds,
    //     arm_u_min,
    //     arm_u_max};

    // arm_optimal_command = fun_service.arm_optimal_controller(arm_opt_control_argument);

    // std::copy(arm_optimal_command[0].nonzeros().begin(), arm_optimal_command[0].nonzeros().end(),
    //           uvms_world[agent_id].force_input.begin() + 6);

    // uvms_world[agent_id].lyapunov_energy = arm_optimal_command.at(1).nonzeros()[0];
    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // /////////////////////////compute base force//////////////////////////////////////////////////////////////////////////////////
    // std::vector<casadi::DM> arm_torques_(uvms_world[agent_id].force_input.begin() + 6,
    //                                      uvms_world[agent_id].force_input.end() - 1);

    // std::vector<double> q_min(joint_min.begin() + 6, joint_min.begin() + 10);
    // std::vector<double> q_max(joint_max.begin() + 6, joint_max.begin() + 10);

    // arm_f_base_argument = {manipulator_states, arm_optimal_command[0], manipulator_parameters, dt, q_min, q_max, gravity, base_gravity, base_To, arm_noise};

    // base_force = fun_service.f_base(arm_f_base_argument);
    // /////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // std::vector<casadi::DM> vehicle_position_state(uvms_position_state.begin(),
    //                                                uvms_position_state.begin() + 6);

    // std::vector<casadi::DM> vehicle_rpy(uvms_position_state.begin() + 3,
    //                                     uvms_position_state.begin() + 6);

    // std::vector<casadi::DM> vehicle_velocity_state(uvms_velocity_state.begin(),
    //                                                uvms_velocity_state.begin() + 6);

    // // Build argument containers for vehicle_H and vehicle_B.
    // vehicle_H_argument = {vehicle_parameters};

    // vehicle_B_argument = {uvms_position_state[2],
    //                       vehicle_rpy,
    //                       vehicle_velocity_state,
    //                       vehicle_parameters,
    //                       base_force.at(0)};

    // uv_H_ = fun_service.vehicle_H(vehicle_H_argument);
    // uv_B_ = fun_service.vehicle_B(vehicle_B_argument);

    // inv_uv_H_ = casadi::DM::solve(uv_H_.at(0), casadi::DM::eye(uv_H_.at(0).size1()));

    // uv_J_ned_argument = {casadi::DM({uvms_world[agent_id].current_position[3],
    //                                  uvms_world[agent_id].current_position[4],
    //                                  uvms_world[agent_id].current_position[5]})};

    // uv_J_ned = fun_service.uv_J_ned(uv_J_ned_argument);

    // uv_J_REF_ned_argument = {casadi::DM({uvms_world[agent_id].XF[3],
    //                                      uvms_world[agent_id].XF[4],
    //                                      uvms_world[agent_id].XF[5]})};
    // uv_J_REF_ned = fun_service.uv_J_ned(uv_J_REF_ned_argument);

    // // Define a1_v and a2_v as vectors of casadi::DM
    // std::vector<casadi::DM> uv_a1_v = {2, 2, 35, 2, 2, 2};
    // std::vector<casadi::DM> uv_a2_v = {0.5, 0.5, 5.5, 0.5, 0.5, 0.5};

    // // Initialize a12_v and ac_v with the same size as a1_v and a2_v
    // std::vector<casadi::DM> uv_a12_v(uv_a1_v.size());
    // std::vector<casadi::DM> uv_ac_v(uv_a1_v.size());

    // // Perform element-wise sqrt and subtraction using casadi::sqrt
    // for (size_t i = 0; i < uv_a1_v.size(); ++i)
    // {
    //     uv_a12_v[i] = casadi::DM::sqrt(uv_a1_v[i] * uv_a2_v[i]) - 1e-8;
    // }

    // // Perform element-wise division and multiply by 2
    // for (size_t i = 0; i < uv_a12_v.size(); ++i)
    // {
    //     uv_ac_v[i] = 2 * (uv_a12_v[i] / uv_a2_v[i]);
    // }

    // // Define epsilon_t correctly
    // double uv_epsilon_t = 0.0;

    // // Prepare optimal_control_params using vertcat as a free function
    // // Flatten the vectors into a single vector of casadi::DM
    // std::vector<casadi::DM> uv_vertcat_args;
    // uv_vertcat_args.insert(uv_vertcat_args.end(), uv_a1_v.begin(), uv_a1_v.end());
    // uv_vertcat_args.insert(uv_vertcat_args.end(), uv_a2_v.begin(), uv_a2_v.end());
    // uv_vertcat_args.insert(uv_vertcat_args.end(), uv_a12_v.begin(), uv_a12_v.end());
    // uv_vertcat_args.insert(uv_vertcat_args.end(), uv_ac_v.begin(), uv_ac_v.end());
    // uv_vertcat_args.emplace_back(uv_epsilon_t); // Convert epsilon_t to casadi::DM implicitly

    // uv_optimal_control_params = casadi::DM::vertcat(uv_vertcat_args);

    // // Convert optimal_control_params to string using the helper function
    // std::string uv_params_str = dm_to_string(uv_optimal_control_params);

    // // Log the optimal_control_params correctly
    // RCLCPP_DEBUG(
    //     logger,
    //     "Got optimal parameters: %s",
    //     uv_params_str.c_str());

    // std::vector<casadi::DM> vehicle_states;
    // vehicle_states.reserve(12);
    // vehicle_states.insert(vehicle_states.end(), vehicle_position_state.begin(), vehicle_position_state.end());
    // vehicle_states.insert(vehicle_states.end(), vehicle_velocity_state.begin(), vehicle_velocity_state.end());

    // std::vector<casadi::DM> vehicle_q_ref(uvms_world[agent_id].XF.begin(), uvms_world[agent_id].XF.begin() + 6);
    // std::vector<casadi::DM> vehicle_dq_ref(uvms_world[agent_id].VF.begin(), uvms_world[agent_id].VF.begin() + 6);
    // std::vector<casadi::DM> vehicle_ddq_ref(uvms_world[agent_id].AF.begin(), uvms_world[agent_id].AF.begin() + 6);

    // std::vector<casadi::DM> vehicle_u_min(uvms_world[agent_id].u_min.begin(), uvms_world[agent_id].u_min.begin() + 6);
    // std::vector<casadi::DM> vehicle_u_max(uvms_world[agent_id].u_max.begin(), uvms_world[agent_id].u_max.begin() + 6);

    // uv_opt_control_argument = {
    //     vehicle_states,
    //     uv_optimal_control_params,
    //     inv_uv_H_,
    //     uv_H_.at(0),
    //     uv_B_.at(0),
    //     vehicle_q_ref,   // q_ref
    //     vehicle_dq_ref,  // dq_ref
    //     vehicle_ddq_ref, // ddq_ref
    //     uv_J_ned.at(0),
    //     uv_J_REF_ned.at(0),
    //     time_seconds,
    //     vehicle_u_min,
    //     vehicle_u_max};

    // uv_optimal_command = fun_service.uv_optimal_controller(uv_opt_control_argument);

    // std::copy(uv_optimal_command[0].nonzeros().begin(), uv_optimal_command[0].nonzeros().end(),
    //           uvms_world[agent_id].force_input.begin());

    // uvms_world[agent_id].lyapunov_energy += uv_optimal_command.at(1).nonzeros()[0];
    // // Log the current Lyapunov energy
    // RCLCPP_DEBUG(logger, "Agent %d Lyapunov energy: %f", agent_id, uvms_world[agent_id].lyapunov_energy);

    return controller_interface::return_type::OK;
};

std::string casadi_uvms::Dynamics::matrixToString(const casadi::Matrix<double> &mat)
{
    std::stringstream ss;
    for (int i = 0; i < mat.size1(); i++)
    {
        for (int j = 0; j < mat.size2(); j++)
        {
            ss << mat(i, j) << " ";
        }
        ss << "\n";
    }
    return ss.str();

    // use like this
    //  // / Convert the first matrix to a string and log it.
    //  std::string matStr = matrixToString(uv_J_ned_test.at(0));
    //  RCLCPP_INFO(logger, "uv_J_ned_test[0]: \n%s", matStr.c_str());
}

void casadi_uvms::Dynamics::simulate(
    const rclcpp::Logger &logger,
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

    ////////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<casadi::DM> arm_state;
    arm_state.reserve(8);
    arm_state.insert(arm_state.end(), arm_position_.begin(), arm_position_.end() - 1);
    arm_state.insert(arm_state.end(), arm_velocity_.begin(), arm_velocity_.end() - 1);

    std::vector<casadi::DM> arm_torques_(uvms_world[agent_id].force_input.begin() + 6,
                                         uvms_world[agent_id].force_input.end() - 1);

    std::vector<double> q_min(joint_min.begin() + 6, joint_min.begin() + 10);
    std::vector<double> q_max(joint_max.begin() + 6, joint_max.begin() + 10);

    casadi::DM payload_weight = 19.62;

    std::vector<casadi::DM> lower_joint_limit(
        joint_min.begin() + 6,
        joint_min.end()
      );
      
      std::vector<casadi::DM> upper_joint_limit(
        joint_max.begin() + 6,
        joint_max.end()
      );
      

    arm_simulate_argument = {arm_state, arm_torques_, dt, gravity, payload_weight, manipulator_parameters, lower_joint_limit, upper_joint_limit};
    arm_sim = fun_service.arm_dynamics(arm_simulate_argument);
    arm_next_states = arm_sim.at(0).nonzeros();
    // arm_base_f_ext = arm_sim.at(1).nonzeros();

    // arm_base_f_ext[1] = -arm_base_f_ext[1];
    // arm_base_f_ext[2] = -arm_base_f_ext[2];

    // arm_base_f_ext[4] = -arm_base_f_ext[4];
    // arm_base_f_ext[5] = -arm_base_f_ext[5];
    arm_base_f_ext = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    ///////////////////////////////////////////////////////////////////////////////////////////////
    std::vector<casadi::DM> uv_state;
    uv_state.reserve(12);
    uv_state.insert(uv_state.end(), vehicle_pose_.begin(), vehicle_pose_.end());
    uv_state.insert(uv_state.end(), vehicle_vel_.begin(), vehicle_vel_.end());

    std::vector<casadi::DM> uv_forces_(uvms_world[agent_id].force_input.begin(),
                                       uvms_world[agent_id].force_input.end() - 5);

    // // uv_forces_…
    // const std::vector<std::string> labels = {"x", "y", "z", "roll", "pitch", "yaw"};
    // double v = static_cast<double>(uv_forces_[5].scalar());
    // RCLCPP_INFO(logger, "%s: %f", labels[5].c_str(), v);

    vehicle_simulate_argument = {uv_state, uv_forces_, vehicle_parameters, dt, arm_base_f_ext};
    vehicle_sim = fun_service.uv_dynamics(vehicle_simulate_argument);
    vehicle_next_states = vehicle_sim.at(0).nonzeros();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    uvms_world[agent_id].next_position[0] = vehicle_next_states[0];
    uvms_world[agent_id].next_position[1] = vehicle_next_states[1];
    uvms_world[agent_id].next_position[2] = vehicle_next_states[2];

    uvms_world[agent_id].next_position[3] = vehicle_next_states[3];
    uvms_world[agent_id].next_position[4] = vehicle_next_states[4];
    uvms_world[agent_id].next_position[5] = vehicle_next_states[5];

    uvms_world[agent_id].next_position[6] = arm_next_states[0];
    uvms_world[agent_id].next_position[7] = arm_next_states[1];
    uvms_world[agent_id].next_position[8] = arm_next_states[2];
    uvms_world[agent_id].next_position[9] = arm_next_states[3];

    uvms_world[agent_id].next_velocity[0] = vehicle_next_states[6];
    uvms_world[agent_id].next_velocity[1] = vehicle_next_states[7];
    uvms_world[agent_id].next_velocity[2] = vehicle_next_states[8];

    uvms_world[agent_id].next_velocity[3] = vehicle_next_states[9];
    uvms_world[agent_id].next_velocity[4] = vehicle_next_states[10];
    uvms_world[agent_id].next_velocity[5] = vehicle_next_states[11];

    uvms_world[agent_id].next_velocity[6] = arm_next_states[4];
    uvms_world[agent_id].next_velocity[7] = arm_next_states[5];
    uvms_world[agent_id].next_velocity[8] = arm_next_states[6];
    uvms_world[agent_id].next_velocity[9] = arm_next_states[7];

    RCLCPP_DEBUG(
        logger,
        "Got velocity commands: %f,%f,%f,%f,  %f,%f,%f,%f,  %f,%f",
        uvms_world[agent_id].next_velocity[0],
        uvms_world[agent_id].next_velocity[1],
        uvms_world[agent_id].next_velocity[2],
        uvms_world[agent_id].next_velocity[3],
        uvms_world[agent_id].next_velocity[4],
        uvms_world[agent_id].next_velocity[5],
        uvms_world[agent_id].next_velocity[6],
        uvms_world[agent_id].next_velocity[7],
        uvms_world[agent_id].next_velocity[8],
        uvms_world[agent_id].next_velocity[9]);
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