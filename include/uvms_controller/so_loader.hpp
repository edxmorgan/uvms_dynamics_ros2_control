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

#ifndef UVMS_CONTROLLER__SO_LOADER_HPP_
#define UVMS_CONTROLLER__SO_LOADER_HPP_

#include <string>
#include <algorithm>
#include <casadi/casadi.hpp>

using namespace casadi;

namespace casadi_uvms
{
    class FunctionLoader
    {

    public:
        Function tester; // uvms casadi tester
        Function decoupled_manipulator_uvms_dynamics; // uvms decoupled_dynamics - manipulator
        Function decoupled_vehicle_uvms_dynamics; // uvms decoupled_dynamics - vehicle
        Function coupled_uvms_dynamics; // uvms coupled_dynamics
        Function manipulator_forward_kinematics; //manipulator forward kinematics
        Function vehicle_position_pid; //vehicle pose pid
        Function vehicle_velocity_pid; //vehicle velocity pid
        FunctionLoader() = default;

        void usage_cplusplus_checks(const std::string &name, const std::string &bin_name, const std::string &node_name);
        /**
         * @brief checks casadi function loader works well
         *
         * @param name Name as in the label assigned to a CasADi Function object
         * @param bin_name File name of the shared library
         */
        Function load_casadi_fun(const std::string &name, const std::string &bin_name);
        /**
         * @brief checks casadi function loader works well
         *
         * @param name Name as in the label assigned to a CasADi Function object
         * @param bin_name File name of the shared library
         */
    };
} // namespace casadi_uvms
#endif // UVMS_CONTROLLER__SO_LOADER_HPP_