#pragma once

#include "ck_utilities_ros2_node/interpolables/Interpolable.hpp"

namespace ck
{
    namespace team254_geometry
    {
        template <class S>
        class State : public ck::math::Interpolable<S>
        {
        public:
           virtual double distance(const S &other) const = 0;
           virtual S add(const S &other) const = 0;
           virtual bool equals(const S &other) = 0;
        };
    }
}