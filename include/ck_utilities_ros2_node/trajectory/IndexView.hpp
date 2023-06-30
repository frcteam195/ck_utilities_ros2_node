#pragma once

#include "ck_utilities_ros2_node/CKMath.hpp"
#include "ck_utilities_ros2_node/team254_geometry/State.hpp"

#include "TrajectorySamplePoint.hpp"
#include "TrajectoryView.hpp"

// #include <iostream>
#include <vector>

namespace ck
{
    namespace trajectory
    {
        template <class S, class T>
        class IndexView : public TrajectoryView<S, T>
        {
            static_assert(std::is_base_of<ck::team254_geometry::State<S>, S>::value, "S must inherit from State<S>");
            static_assert(std::is_base_of<ck::team254_geometry::State<T>, T>::value, "T must inherit from State<T>");

        protected:
            Trajectory<S, T> *trajectory_;
            bool is_initialized;
        public:
            IndexView()
            {
                is_initialized = false;
            }

            void initialize(Trajectory<S, T>* traj)
            {
                trajectory_ = traj;
                is_initialized = true;
            }

            TrajectorySamplePoint<S, T> sample(double index) override { return trajectory_->getInterpolated(index); }
            double last_interpolant() const override { return ck::math::max(0, trajectory_->length() - 1); }
            double first_interpolant() const override { return 0.0; }
            Trajectory<S, T> trajectory() override { return *trajectory_; }
        };
    } // namespace trajectory
} // namespace ck
