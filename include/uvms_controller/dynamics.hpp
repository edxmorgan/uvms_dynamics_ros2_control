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

namespace casadi_uvms
{
    class Dynamics
    {
    private:
        // Store the dynamics function for the whole body robot
        casadi_uvms::FunctionLoader fun_service;
        std::vector<DM> arm_simulate_argument;
        std::vector<DM> arm_sim;
        std::vector<DM> vehicle_simulate_argument;
        std::vector<DM> vehicle_sim;

    public:
        struct Model
        {
            int id;
            std::vector<DM> current_position = std::vector<DM>(11);
            std::vector<double> next_position = std::vector<double>(11);
            std::vector<DM> current_velocity = std::vector<DM>(10);
            std::vector<double> next_velocity = std::vector<double>(10);
            std::vector<double> force_input = std::vector<double>(10);
            std::vector<DM> model_p = std::vector<DM>(12);
            std::vector<DM> flow_velocity = std::vector<DM>(6);
            std::vector<double> uvms_base_TF_;
            std::vector<int> poseSubscriber;
            std::vector<int> poseCommander;
            std::vector<int> velSubscriber;
            std::vector<int> velCommander;
            std::vector<int> effortCommander;
            bool grabber_open;
        };

        std::vector<Model> uvms_world{}; // vector for future enhancement

        DM dt;
        Dynamics()
        {
            init_dynamics(); // Automatically call init_dynamics upon instantiation
        }

        void init_dynamics();

        void coupled_simulate(int &agent_id);

        void decoupled_simulate(int &agent_id);
    };
} // namespace casadi_uvms

#endif // UVMS_CONTROLLER__DYNAMICS_HPP_
