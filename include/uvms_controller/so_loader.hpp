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
        Function uv_dynamics; // standalone uv dynamics
        Function arm_dynamics; // arm standalone dynamics
        Function forward_kinematics; // forward kinematics
        Function pid_controller; // whole-body PID controller
        Function uv_G; // restoring force acting on uv
        Function uv_J_ned; //NED Transform matrix
        Function arm_H; // arm inertia matrix dynamics
        Function arm_B; // arm bias vector dynamics
        Function vehicle_H; // uv inertia matrix dynamics
        Function vehicle_B; // uv bias vector dynamics
        Function uv_optimal_controller; // uv optimal controller
        Function arm_optimal_controller; // arm optimal controller
        Function f_base; // force at base
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