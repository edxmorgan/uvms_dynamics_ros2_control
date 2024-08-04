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

#ifndef UVMS_CONTROLLER__UVMS_INTERFACE_HPP_
#define UVMS_CONTROLLER__UVMS_INTERFACE_HPP_

#include <iostream>
#include <map>
#include <string>

namespace uvms_controller
{
    // Define enums for pose topic interfaces
    enum class PoseTopic
    {
        POSITION_X = 0,
        POSITION_Y = 1,
        POSITION_Z = 2,
        ORIENTATION_W = 3,
        ORIENTATION_X = 4,
        ORIENTATION_Y = 5,
        ORIENTATION_Z = 6,
        FILTERED_POSITION_J_0 = 13,
        FILTERED_POSITION_J_1 = 15,
        FILTERED_POSITION_J_2 = 17,
        FILTERED_POSITION_J_3 = 19,
        FILTERED_POSITION_ENDEFFECTOR = 21
    };

    // Define enums for velocity topic interfaces
    enum class VelocityTopic
    {
        VELOCITY_U = 7,
        VELOCITY_V = 8,
        VELOCITY_W = 9,
        VELOCITY_P = 10,
        VELOCITY_Q = 11,
        VELOCITY_R = 12,
        FILTERED_VELOCITY_J_0 = 14,
        FILTERED_VELOCITY_J_1 = 16,
        FILTERED_VELOCITY_J_2 = 18,
        FILTERED_VELOCITY_J_3 = 20,
        FILTERED_VELOCITY_ENDEFFECTOR = 22
    };

    // Define enums for pose command interfaces
    enum class PoseCommand
    {
        POSITION_X = 0,
        POSITION_Y = 1,
        POSITION_Z = 2,
        ORIENTATION_W = 3,
        ORIENTATION_X = 4,
        ORIENTATION_Y = 5,
        ORIENTATION_Z = 6,
        POSITION_J_0 = 19,
        POSITION_J_1 = 22,
        POSITION_J_2 = 25,
        POSITION_J_3 = 28,
        POSITION_ENDEFFECTOR = 31
    };

    // Define enums for velocity command interfaces
    enum class VelocityCommand
    {
        VELOCITY_U = 7,
        VELOCITY_V = 8,
        VELOCITY_W = 9,
        VELOCITY_P = 10,
        VELOCITY_Q = 11,
        VELOCITY_R = 12,
        VELOCITY_J_0 = 20,
        VELOCITY_J_1 = 23,
        VELOCITY_J_2 = 26,
        VELOCITY_J_3 = 29,
        VELOCITY_ENDEFFECTOR = 32
    };

    // Define enums for effort command interfaces
    enum class EffortCommand
    {
        FORCE_X = 13,
        FORCE_Y = 14,
        FORCE_Z = 15,
        TORQUE_X = 16,
        TORQUE_Y = 17,
        TORQUE_Z = 18,
        EFFORT_J_0 = 21,
        EFFORT_J_1 = 24,
        EFFORT_J_2 = 27,
        EFFORT_J_3 = 30,
        EFFORT_ENDEFFECTOR = 33
    };

    namespace vehicle
    {
        enum class VehiclePositionTopic
        {
            POSITION_X = static_cast<int>(PoseTopic::POSITION_X),
            POSITION_Y = static_cast<int>(PoseTopic::POSITION_Y),
            POSITION_Z = static_cast<int>(PoseTopic::POSITION_Z),
            ORIENTATION_W = static_cast<int>(PoseTopic::ORIENTATION_W),
            ORIENTATION_X = static_cast<int>(PoseTopic::ORIENTATION_X),
            ORIENTATION_Y = static_cast<int>(PoseTopic::ORIENTATION_Y),
            ORIENTATION_Z = static_cast<int>(PoseTopic::ORIENTATION_Z)
        };

        enum class VehicleVelocityTopic
        {
            VELOCITY_U = static_cast<int>(VelocityTopic::VELOCITY_U),
            VELOCITY_V = static_cast<int>(VelocityTopic::VELOCITY_V),
            VELOCITY_W = static_cast<int>(VelocityTopic::VELOCITY_W),
            VELOCITY_P = static_cast<int>(VelocityTopic::VELOCITY_P),
            VELOCITY_Q = static_cast<int>(VelocityTopic::VELOCITY_Q),
            VELOCITY_R = static_cast<int>(VelocityTopic::VELOCITY_R)
        };

        enum class VehiclePoseCommand
        {
            POSITION_X = static_cast<int>(PoseCommand::POSITION_X),
            POSITION_Y = static_cast<int>(PoseCommand::POSITION_Y),
            POSITION_Z = static_cast<int>(PoseCommand::POSITION_Z),
            ORIENTATION_W = static_cast<int>(PoseCommand::ORIENTATION_W),
            ORIENTATION_X = static_cast<int>(PoseCommand::ORIENTATION_X),
            ORIENTATION_Y = static_cast<int>(PoseCommand::ORIENTATION_Y),
            ORIENTATION_Z = static_cast<int>(PoseCommand::ORIENTATION_Z)
        };

        enum class VehicleVelocityCommand
        {
            VELOCITY_U = static_cast<int>(VelocityCommand::VELOCITY_U),
            VELOCITY_V = static_cast<int>(VelocityCommand::VELOCITY_V),
            VELOCITY_W = static_cast<int>(VelocityCommand::VELOCITY_W),
            VELOCITY_P = static_cast<int>(VelocityCommand::VELOCITY_P),
            VELOCITY_Q = static_cast<int>(VelocityCommand::VELOCITY_Q),
            VELOCITY_R = static_cast<int>(VelocityCommand::VELOCITY_R)
        };

        enum class VehicleEffortCommand
        {
            FORCE_X = static_cast<int>(EffortCommand::FORCE_X),
            FORCE_Y = static_cast<int>(EffortCommand::FORCE_Y),
            FORCE_Z = static_cast<int>(EffortCommand::FORCE_Z),
            TORQUE_X = static_cast<int>(EffortCommand::TORQUE_X),
            TORQUE_Y = static_cast<int>(EffortCommand::TORQUE_Y),
            TORQUE_Z = static_cast<int>(EffortCommand::TORQUE_Z)
        };
    } // namespace vehicle

    namespace manipulator
    {
        enum class Joint0
        {
            PoseTopicInterface = static_cast<int>(PoseTopic::FILTERED_POSITION_J_0),
            VelocityTopicInterface = static_cast<int>(VelocityTopic::FILTERED_VELOCITY_J_0),
            PoseCommandInterface = static_cast<int>(PoseCommand::POSITION_J_0),
            VelocityCommandInterface = static_cast<int>(VelocityCommand::VELOCITY_J_0),
            EffortCommandInterface = static_cast<int>(EffortCommand::EFFORT_J_0)
        };

        enum class Joint1
        {
            PoseTopicInterface = static_cast<int>(PoseTopic::FILTERED_POSITION_J_1),
            VelocityTopicInterface = static_cast<int>(VelocityTopic::FILTERED_VELOCITY_J_1),
            PoseCommandInterface = static_cast<int>(PoseCommand::POSITION_J_1),
            VelocityCommandInterface = static_cast<int>(VelocityCommand::VELOCITY_J_1),
            EffortCommandInterface = static_cast<int>(EffortCommand::EFFORT_J_1)
        };

        enum class Joint2
        {
            PoseTopicInterface = static_cast<int>(PoseTopic::FILTERED_POSITION_J_2),
            VelocityTopicInterface = static_cast<int>(VelocityTopic::FILTERED_VELOCITY_J_2),
            PoseCommandInterface = static_cast<int>(PoseCommand::POSITION_J_2),
            VelocityCommandInterface = static_cast<int>(VelocityCommand::VELOCITY_J_2),
            EffortCommandInterface = static_cast<int>(EffortCommand::EFFORT_J_2)
        };

        enum class Joint3
        {
            PoseTopicInterface = static_cast<int>(PoseTopic::FILTERED_POSITION_J_3),
            VelocityTopicInterface = static_cast<int>(VelocityTopic::FILTERED_VELOCITY_J_3),
            PoseCommandInterface = static_cast<int>(PoseCommand::POSITION_J_3),
            VelocityCommandInterface = static_cast<int>(VelocityCommand::VELOCITY_J_3),
            EffortCommandInterface = static_cast<int>(EffortCommand::EFFORT_J_3)
        };

        enum class Endeffector
        {
            PoseTopicInterface = static_cast<int>(PoseTopic::FILTERED_POSITION_ENDEFFECTOR),
            VelocityTopicInterface = static_cast<int>(VelocityTopic::FILTERED_VELOCITY_ENDEFFECTOR),
            PoseCommandInterface = static_cast<int>(PoseCommand::POSITION_ENDEFFECTOR),
            VelocityCommandInterface = static_cast<int>(VelocityCommand::VELOCITY_ENDEFFECTOR),
            EffortCommandInterface = static_cast<int>(EffortCommand::EFFORT_ENDEFFECTOR)
        };
    } // namespace manipulator
} // namespace uvms_controller

#endif // UVMS_CONTROLLER__UVMS_INTERFACE_HPP_
