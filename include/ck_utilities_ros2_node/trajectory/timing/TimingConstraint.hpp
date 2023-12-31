#pragma once

#include "ck_utilities_ros2_node/CKMath.hpp"
#include "ck_utilities_ros2_node/team254_geometry/State.hpp"
#include "ck_utilities_ros2_node/team254_geometry/Geometry.hpp"
#include "MinMaxAcceleration.hpp"

#include <type_traits>

namespace ck
{
    namespace trajectory
    {
        namespace timing
        {
            enum ConstraintType
            {
                NONE,
                CONDITIONAL_VELOCITY,
                CONDITIONAL_ACCEL,
                VELOCITY_LIMIT_REGION
            };

            template <class S>
            class TimingConstraint
            {
                static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State<S>");

            public:
                TimingConstraint(ConstraintType type = ConstraintType::NONE,
                                 ck::team254_geometry::Translation2d min_corner = ck::team254_geometry::Translation2d(0, 0),
                                 ck::team254_geometry::Translation2d max_corner = ck::team254_geometry::Translation2d(0, 0),
                                 double velocity_limit = 0.0)
                {
                    type_ = type;
                    min_corner_ = min_corner;
                    max_corner_ = max_corner;
                    velocity_limit_ = velocity_limit;
                }

                ~TimingConstraint() {}

                virtual double getMaxVelocity(const S &state) const
                {
                    switch (type_)
                    {
                    case ConstraintType::NONE:
                        return 0;
                        break;

                    case ConstraintType::CONDITIONAL_VELOCITY:
                        if (state.getTranslation().x() >= 24.0)
                        {
                            return 5.0;
                        }
                        else
                        {
                            return ck::math::POS_INF;
                        }
                        break;

                    case ConstraintType::CONDITIONAL_ACCEL:
                        return ck::math::POS_INF;
                        break;

                    case ConstraintType::VELOCITY_LIMIT_REGION:
                        ck::team254_geometry::Translation2d translation = state.getTranslation();
                        if (translation.x() <= max_corner_.x() && translation.x() >= min_corner_.x() &&
                            translation.y() <= max_corner_.y() && translation.y() >= min_corner_.y())
                        {
                            return velocity_limit_;
                        }
                        return ck::math::POS_INF;
                        break;
                    }

                    // Default return to suppress [-Wreturn-type].
                    return 0.0;
                }

                virtual MinMaxAcceleration getMinMaxAcceleration(const S &state, double velocity) const
                {
                    // TODO: Why is state unused?
                    (void)state;

                    switch (type_)
                    {
                    case ConstraintType::NONE:
                        // Never gets called
                        // return MinMaxAcceleration::kNoLimits;
                        return MinMaxAcceleration();
                        break;

                    case ConstraintType::VELOCITY_LIMIT_REGION:
                        // return MinMaxAcceleration::kNoLimits;
                        return MinMaxAcceleration();
                        break;

                    case ConstraintType::CONDITIONAL_VELOCITY:
                        return MinMaxAcceleration(ck::math::NEG_INF, ck::math::POS_INF);
                        break;

                    case ConstraintType::CONDITIONAL_ACCEL:
                        return MinMaxAcceleration(-10.0, 10.0 / velocity);
                        break;
                    }

                    // Default return to suppress [-Wreturn-type].
                    // return MinMaxAcceleration::kNoLimits;
                    return MinMaxAcceleration();
                }

            private:
                ConstraintType type_;
                ck::team254_geometry::Translation2d min_corner_;
                ck::team254_geometry::Translation2d max_corner_;
                double velocity_limit_;
            };
        } // namespace timing
    } // namespace trajectory
} // namespace ck