#pragma once

#include "ck_utilities_ros2_node/CKMath.hpp"
#include "ck_utilities_ros2_node/team254_geometry/Geometry.hpp"
#include "ck_utilities_ros2_node/team254_geometry/QuinticHermiteSpline.hpp"
#include "ck_utilities_ros2_node/trajectory/TrajectoryPoint.hpp"

#include <cmath>
#include <vector>

namespace ck
{
    namespace team254_geometry
    {
        class SplineGenerator
        {
        private:
            static constexpr double kMaxDX = 2.0;     //inches
            static constexpr double kMaxDY = 0.05;    //inches
            static constexpr double kMaxDTheta = 0.1; //radians!
            static constexpr double kMinSampleSize = 1;

            // static void getSegmentArc(QuinticHermiteSpline &s, std::vector<Pose2dWithCurvature> &rv, double t0, double t1, double maxDx, double maxDy, double maxDTheta);
            static void getSegmentArc(QuinticHermiteSpline &s,
                                      std::vector<Rotation2d> &headings,
                                      std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> &rv,
                                      double t0,
                                      double t1,
                                      double maxDx,
                                      double maxDy,
                                      double maxDTheta,
                                      double totalTime,
									  int depth=0);

        public:
            /**
             * Converts a spline into a list of Twist2d's.
             *
             * @param s  the spline to parametrize
             * @param t0 starting percentage of spline to parametrize
             * @param t1 ending percentage of spline to parametrize
             * @return list of Pose2dWithCurvature that approximates the original spline
             */
            // static std::vector<ck::team254_geometry::Pose2dWithCurvature> parameterizeSpline(QuinticHermiteSpline &s, double maxDx, double maxDy, double maxDTheta, double t0, double t1);
            static std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> parameterizeSpline(QuinticHermiteSpline &s,
                                                                                                                std::vector<Rotation2d> &headings,
                                                                                                                double maxDx,
                                                                                                                double maxDy,
                                                                                                                double maxDTheta,
                                                                                                                double t0,
                                                                                                                double t1);

            /**
             * Convenience function to parametrize a spline from t 0 to 1
             */
            // static std::vector<ck::team254_geometry::Pose2dWithCurvature> parameterizeSpline(QuinticHermiteSpline &s);
            static std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> parameterizeSpline(QuinticHermiteSpline &s, std::vector<Rotation2d> &headings);
            // static std::vector<ck::team254_geometry::Pose2dWithCurvature> parameterizeSpline(QuinticHermiteSpline &s, double maxDx, double maxDy, double maxDTheta);
            static std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> parameterizeSpline(QuinticHermiteSpline &s,
                                                                                                                std::vector<Rotation2d> &headings,
                                                                                                                double maxDx,
                                                                                                                double maxDy,
                                                                                                                double maxDTheta);
            // static std::vector<ck::team254_geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<QuinticHermiteSpline> &splines);
            static std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> parameterizeSplines(std::vector<QuinticHermiteSpline> &splines, std::vector<Rotation2d> &headings);
            // static std::vector<ck::team254_geometry::Pose2dWithCurvature> parameterizeSplines(std::vector<QuinticHermiteSpline> &splines, double maxDx, double maxDy, double maxDTheta);
            static std::vector<trajectory::TrajectoryPoint<Pose2dWithCurvature, Rotation2d>> parameterizeSplines(std::vector<QuinticHermiteSpline> &splines,
                                                                                                                 std::vector<Rotation2d> &headings,
                                                                                                                 double maxDx,
                                                                                                                 double maxDy,
                                                                                                                 double maxDTheta);
        };
    } // namespace team254_geometry
} // namespace ck