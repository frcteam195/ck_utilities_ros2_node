#pragma once

#include "ck_utilities_ros2_node/team254_geometry/Pose2dWithCurvature.hpp"
#include "ck_utilities_ros2_node/trajectory/Trajectory.hpp"
#include "ck_utilities_ros2_node/trajectory/timing/TimedState.hpp"

namespace ck
{
    namespace trajectory
    {
        class MirroredTrajectory
        {
        public:
            // MirroredTrajectory(trajectory::Trajectory<trajectory::timing::TimedState<team254_geometry::Pose2dWithCurvature> > right);

            // trajectory::Trajectory<trajectory::timing::TimedState<team254_geometry::Pose2dWithCurvature> > get(bool left = false);

            // trajectory::Trajectory<trajectory::timing::TimedState<team254_geometry::Pose2dWithCurvature> > left;
            // trajectory::Trajectory<trajectory::timing::TimedState<team254_geometry::Pose2dWithCurvature> > right;
        };
    } // namespace trajectory
} // namespace ck
