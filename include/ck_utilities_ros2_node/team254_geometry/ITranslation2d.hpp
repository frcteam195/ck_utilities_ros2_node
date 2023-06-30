#pragma once

#include "ck_utilities_ros2_node/team254_geometry/State.hpp"

namespace ck
{
    namespace team254_geometry
    {
        class Translation2d;

        template <class S>
        class ITranslation2d : public State<S>
        {
        public:
            virtual Translation2d getTranslation() const = 0;
        };
    } // namespace team254_geometry
} // namespace ck