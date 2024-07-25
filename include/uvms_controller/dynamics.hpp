#ifndef UVMS_CONTROLLER__DYNAMICS_HPP_
#define UVMS_CONTROLLER__DYNAMICS_HPP_

#include <string>
#include <algorithm>
#include <casadi/casadi.hpp>

using namespace casadi;

namespace casadi_uvms
{
    class Dynamics
    {

    public:
        Function dynamics; // uvms dynamics

        Dynamics() = default;

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
} // namespace casadi_reach_alpha_5
#endif // ROS2_CONTROL_BLUE_REACH_5__JOINT_HPP_