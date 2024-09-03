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
};

void casadi_uvms::Dynamics::coupled_simulate(int &agent_id) {

};

void casadi_uvms::Dynamics::decoupled_simulate(int &agent_id)
{
    // vehicle_sim = fun_service.decoupled_vehicle_uvms_dynamics;
    // fun_service.manipulator_forward_kinematics;

    std::vector<casadi::DM> last_four_position(uvms_world[agent_id].current_position.end() - 4,
                                               uvms_world[agent_id].current_position.end());

    std::vector<casadi::DM> last_four_velocity(uvms_world[agent_id].current_velocity.end() - 4,
                                               uvms_world[agent_id].current_velocity.end());

    std::vector<casadi::DM> last_four_forces(uvms_world[agent_id].force_input.end() - 4,
                                               uvms_world[agent_id].force_input.end());

    simulate_argument = {last_four_position,
                         last_four_velocity,
                         last_four_forces,
                         uvms_world[agent_id].model_p,
                         uvms_world[agent_id].dt};

    arm_sim = fun_service.decoupled_manipulator_uvms_dynamics(simulate_argument);

    // // uvms_world[agent_id].next_position[0] = vehicle_sim.at(0).nonzeros()[0];
    // // uvms_world[agent_id].next_position[1] = vehicle_sim.at(0).nonzeros()[1];
    // // uvms_world[agent_id].next_position[2] = vehicle_sim.at(0).nonzeros()[2];

    // // uvms_world[agent_id].next_position[3] = vehicle_sim.at(0).nonzeros()[3];
    // // uvms_world[agent_id].next_position[4] = vehicle_sim.at(0).nonzeros()[0];
    // // uvms_world[agent_id].next_position[5] = vehicle_sim.at(0).nonzeros()[1];
    // // uvms_world[agent_id].next_position[6] = vehicle_sim.at(0).nonzeros()[1];

    uvms_world[agent_id].next_position[7] = arm_sim.at(0).nonzeros()[0];
    uvms_world[agent_id].next_position[8] = arm_sim.at(0).nonzeros()[1];
    uvms_world[agent_id].next_position[9] = arm_sim.at(0).nonzeros()[2];
    uvms_world[agent_id].next_position[10] = arm_sim.at(0).nonzeros()[3];

    // // uvms_world[agent_id].next_velocity[0] = vehicle_sim.at(0).nonzeros()[4];
    // // uvms_world[agent_id].next_velocity[1] = vehicle_sim.at(0).nonzeros()[5];
    // // uvms_world[agent_id].next_velocity[2] = vehicle_sim.at(0).nonzeros()[6];
    // // uvms_world[agent_id].next_velocity[3] = vehicle_sim.at(0).nonzeros()[7];
    // // uvms_world[agent_id].next_velocity[4] = vehicle_sim.at(0).nonzeros()[4];
    // // uvms_world[agent_id].next_velocity[5] = vehicle_sim.at(0).nonzeros()[5];

    uvms_world[agent_id].next_velocity[6] = arm_sim.at(0).nonzeros()[4];
    uvms_world[agent_id].next_velocity[7] = arm_sim.at(0).nonzeros()[5];
    uvms_world[agent_id].next_velocity[8] = arm_sim.at(0).nonzeros()[6];
    uvms_world[agent_id].next_velocity[9] = arm_sim.at(0).nonzeros()[7];
};
