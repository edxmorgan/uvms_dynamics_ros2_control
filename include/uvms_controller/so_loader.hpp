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
        Function uvms_dynamics; // uvms dynamics
        Function q2euler; //quaternion to euler function
        Function euler2q; //euler 2 quaternion function
        Function forward_kinematics; // forward kinematics
        Function pid_controller; // whole-body PID controller
        Function uv_G; // restoring force acting on uv
        Function uv_J_ned; //NED Transform matrix
        Function uvms_H; // uvms inertia matrix dynamics
        Function uvms_B; // uvms bias vector dynamics
        Function uvms_J_ned; //NED Transform matrix for whole-body
        Function uvms_optimal_controller; // whole-body optimal controller
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