#pragma once

#include "ck_utilities_ros2_node/planners/DriveMotionPlanner.hpp"
#include "ck_utilities_ros2_node/team254_swerve/SwerveModuleState.hpp"

#include <vector>

namespace ck
{
    namespace team254_swerve
    {
        class SwerveSetpoint
        {
        public:
            SwerveSetpoint(planners::ChassisSpeeds chassisSpeeds, std::vector<SwerveModuleState> initialStates)
            {
                mChassisSpeeds = chassisSpeeds;
                mModuleStates = initialStates;
            }

            friend std::ostream& operator<<(std::ostream& os, const SwerveSetpoint& setpoint)
            {
                os << setpoint.mChassisSpeeds << "\n";
                for (size_t i = 0; i < setpoint.mModuleStates.size(); i++)
                {
                    os << "  " << setpoint.mModuleStates.at(i) << "\n";
                }
                return os;
            }

            planners::ChassisSpeeds mChassisSpeeds;
            std::vector<SwerveModuleState> mModuleStates;
        };
    } // namespace team254_swerve
} // namespace ck
