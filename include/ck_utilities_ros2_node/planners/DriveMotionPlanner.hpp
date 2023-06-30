#pragma once

#include "ck_utilities_ros2_node/CKMath.hpp"
#include "ck_utilities_ros2_node/Constants.hpp"
#include "ck_utilities_ros2_node/planners/ChassisSpeeds.hpp"
#include "ck_utilities_ros2_node/team254_geometry/Geometry.hpp"
#include "ck_utilities_ros2_node/physics/DifferentialDrive.hpp"
#include "ck_utilities_ros2_node/trajectory/Lookahead.hpp"
#include "ck_utilities_ros2_node/trajectory/Trajectory.hpp"
#include "ck_utilities_ros2_node/trajectory/TrajectoryIterator.hpp"
#include "ck_utilities_ros2_node/trajectory/timing/TimingConstraint.hpp"
#include "ck_utilities_ros2_node/trajectory/timing/TimedState.hpp"

using namespace ck::trajectory;
using namespace ck::trajectory::timing;
using namespace ck::team254_geometry;

namespace ck
{
    namespace planners
    {

        class DriveMotionPlanner
        {
        public:
            enum FollowerType
            {
                FEEDFORWARD_ONLY,
                PURE_PURSUIT,
                PID,
            };

            DriveMotionPlanner(double max_translational_velocity=3.0);

            void setTrajectory(TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> trajectory);
            void reset(void);

            Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generateTrajectory(bool reversed,
                                                                                                   std::vector<Pose2d> waypoints,
                                                                                                   std::vector<Rotation2d> headings,
                                                                                                   double maximumVelocity,   // Inches per Second
                                                                                                   double maximumAcceleration, // Inches per Second^2
                                                                                                   double maximumVoltage);

            Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generateTrajectory(bool reversed,
                                                                                                   std::vector<Pose2d> waypoints,
                                                                                                   std::vector<Rotation2d> headings,
                                                                                                   double maximumVelocity,   // Inches per Second
                                                                                                   double maximumAcceleration, // Inches per Second^2
                                                                                                   double maximumDeceleration,
                                                                                                   bool apply_smoothing,
                                                                                                   double maximumVoltage);

            Trajectory<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> generateTrajectory(bool reversed,
                                                                                                   std::vector<Pose2d> waypoints,
                                                                                                   std::vector<Rotation2d> headings,
                                                                                                   double startVelocity, // Inches per Second
                                                                                                   double endVelocity, // Inches per Second
                                                                                                   double maximumVelocity, // Inches per Second
                                                                                                   double maximumAcceleration, // Inches per Second^2
                                                                                                   double maximumDeceleration,
                                                                                                   bool apply_smoothing,
                                                                                                   double maximumVoltage);

            ChassisSpeeds updatePurePursuit(Pose2d current_state, double feedforwardOmegaRadiansPerSecond);
            ChassisSpeeds update(double timestamp, Pose2d current_state);

            bool isDone(void);

            double getCurrentProgress();

            Translation2d getTranslationalError();
            Rotation2d getHeadingError();

            double distance(Pose2d current_state, double additional_progress);

            TimedState<Pose2dWithCurvature> getPathSetpoint();
            TimedState<Rotation2d> getHeadingSetpoint();

            void setFollowerType(FollowerType type);

        private:
            const double kMaxVelocityMetersPerSecond;

            static constexpr double kMaxDx = 2.0;
            static constexpr double kMaxDy = 0.25;
            // static constexpr double kMaxDtheta = 0.0872665; // 5 Degrees
            double kMaxDTheta = math::deg2rad(1.0);
            static constexpr double kMaxCentripetalAccel = 60.0;

            FollowerType mFollowerType = FollowerType::PURE_PURSUIT;

            double defaultCook = 0;
            bool useDefaultCook = true;
            double defaultServe = 0;
            bool useDefaultServe = false;

            TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>> *mCurrentTrajectory = new TrajectoryIterator<TimedState<Pose2dWithCurvature>, TimedState<Rotation2d>>();

            bool mIsReversed = false;
            double mLastTime = math::POS_INF;

            TimedState<Pose2dWithCurvature> *mLastPathSetpoint = nullptr;
            TimedState<Pose2dWithCurvature> *mPathSetpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature::identity());
            TimedState<Rotation2d> *mHeadingSetpoint = new TimedState<Rotation2d>(Rotation2d::identity());
            TimedState<Rotation2d> *mLastHeadingSetpoint = new TimedState<Rotation2d>(Rotation2d::identity());

            Pose2d *mError = new Pose2d(Pose2d::identity());

            Translation2d *mTranslationError = new Translation2d(Translation2d::identity());
            Rotation2d *mHeadingError = new Rotation2d(Rotation2d::identity());
            Rotation2d *mInitialHeading = new Rotation2d(Rotation2d::identity());
            Rotation2d *mRotationDiff = new Rotation2d(Rotation2d::identity());

            Pose2d *mCurrentState = new Pose2d(Pose2d::identity());

            double mCurrentTrajectoryLength = 0.0;
            double mTotalTime = math::POS_INF;
            double mStartTime = math::POS_INF;
            double mDTheta = 0.0;

            ChassisSpeeds *mOutput = new ChassisSpeeds();

            Lookahead mSpeedLookahead;

            double mDt = 0.0;
            double lastProgress = 0.0;
        };

    } // namespace planners
} // namespace ck