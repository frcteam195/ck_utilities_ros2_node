#include "ck_utilities_ros2_node/team254_geometry/Rotation2d.hpp"

namespace ck
{
    namespace team254_geometry
    {
        const Rotation2d &Rotation2d::identity()
        {
            static Rotation2d kIdentity;
            return kIdentity;
        }

        const Rotation2d &Rotation2d::kPi()
        {
            static Rotation2d kPi(math::PI, false);
            return kPi;
        }

        const Rotation2d &Rotation2d::kHalfPi()
        {
            static Rotation2d kHalfPi(math::PI / 2.0, false);
            return kHalfPi;
        }

        Rotation2d::Rotation2d() : cos_angle(1), sin_angle(0), radians_(0) {}
        Rotation2d::Rotation2d(double x, double y, bool normalize)
        {
            if (normalize)
            {
                // From trig, we know that sin^2 + cos^2 == 1, but as we do math on this object we might accumulate rounding errors.
                // Normalizing forces us to re-scale the sin and cos to reset rounding errors.
                double magnitude = std::hypot(x, y);
                if (magnitude > ck::math::kEpsilon)
                {
                    sin_angle = y / magnitude;
                    cos_angle = x / magnitude;
                }
                else
                {
                    sin_angle = 0;
                    cos_angle = 1;
                }
            }
            else
            {
                cos_angle = x;
                sin_angle = y;
            }
            calcRadians();
        }
        Rotation2d::Rotation2d(double radians, bool normalize)
        {
            if (normalize)
            {
                radians = WrapRadians(radians);
            }
            radians_ = radians;
            calcTrig();
        }
        Rotation2d::Rotation2d(double x, double y, double radians)
        {
            cos_angle = x;
            sin_angle = y;
            radians_ = radians;
        }
        Rotation2d::Rotation2d(const Translation2d &direction, bool normalize)
        {
            Rotation2d(direction.x(), direction.y(), normalize);
        }

        bool Rotation2d::operator==(const Rotation2d &obj) const
        {
            return distance(obj) < ck::math::kEpsilon;
        }

        std::ostream &operator<<(std::ostream &os, const Rotation2d &r2d)
        {
            os << std::setprecision(3) << "(" << r2d.getDegrees() << " deg)";
            return os;
        }

        Rotation2d Rotation2d::fromRadians(double angle_radians)
        {
            return Rotation2d(std::cos(angle_radians), std::sin(angle_radians), false);
        }
        Rotation2d Rotation2d::fromDegrees(double angle_degrees)
        {
            return fromRadians(ck::math::deg2rad(angle_degrees));
        }

        double Rotation2d::cos() const
        {
            return cos_angle;
        }
        double Rotation2d::sin() const
        {
            return sin_angle;
        }
        double Rotation2d::tan() const
        {
            if (std::abs(cos_angle) < ck::math::kEpsilon)
            {
                if (sin_angle >= 0.0)
                {
                    return ck::math::POS_INF;
                }
                else
                {
                    return ck::math::NEG_INF;
                }
            }
            return sin_angle / cos_angle;
        }
        double Rotation2d::getRadians() const
        {
            return radians_;
        }
        double Rotation2d::getDegrees() const
        {
            return ck::math::rad2deg(getRadians());
        }
        Rotation2d Rotation2d::unaryMinus() const
        {
            return Rotation2d(-radians_, true);
        }
        Rotation2d Rotation2d::minus(const Rotation2d &other) const
        {
            return rotateBy(other.unaryMinus());
        }
        Rotation2d Rotation2d::times(double scalar) const
        {
            return Rotation2d(radians_ * scalar, true);
        }
        Rotation2d Rotation2d::rotateBy(const Rotation2d &other) const
        {
            return Rotation2d(cos_angle * other.cos_angle - sin_angle * other.sin_angle,
                              cos_angle * other.sin_angle + sin_angle * other.cos_angle, true);
        }
        Rotation2d Rotation2d::mirror() const
        {
            return fromRadians(-radians_);
        }
        Rotation2d Rotation2d::normal() const
        {
            return Rotation2d(-sin_angle, cos_angle, false);
        }
        Rotation2d Rotation2d::inverse() const
        {
            return Rotation2d(cos_angle, -sin_angle, false);
        }
        Rotation2d Rotation2d::flip() const
        {
            return Rotation2d(-cos_angle, -sin_angle, false);
        }
        bool Rotation2d::isParallel(const Rotation2d &other) const
        {
            return ck::math::epsilonEquals(Translation2d::cross(toTranslation(), other.toTranslation()), 0.0);
        }
        bool Rotation2d::isParallelLoose(const Rotation2d &other, double epsilon) const
        {
            return ck::math::epsilonEquals(Translation2d::cross(toTranslation(), other.toTranslation()), 0.0, epsilon);
        }
        Translation2d Rotation2d::toTranslation() const
        {
            return Translation2d(cos_angle, sin_angle);
        }
        Rotation2d Rotation2d::interpolate(const Rotation2d &other, double interpFactor) const
        {
            if (interpFactor <= 0)
            {
                return Rotation2d(*this);
            }
            else if (interpFactor >= 1)
            {
                return Rotation2d(other);
            }
            double angle_diff = inverse().rotateBy(other).getRadians();
            return rotateBy(Rotation2d::fromRadians(angle_diff * interpFactor));
        }

        double Rotation2d::distance(const Rotation2d &other) const
        {
            return inverse().rotateBy(other).getRadians();
        }

        Rotation2d Rotation2d::getRotation() const
        {
            return *this;
        }

        bool Rotation2d::equals(const Rotation2d &other) {
            return distance(other) < ck::math::kEpsilon;
        }

        Rotation2d Rotation2d::add(const Rotation2d &other) const
        {
            return rotateBy(other);
        }

        double Rotation2d::WrapRadians(double radians) const
        {
            double k2Pi = 2.0 * math::PI;
            radians = std::fmod(radians, k2Pi);
            radians = std::fmod(radians + k2Pi, k2Pi);
            if (radians > math::PI)
            {
                radians -= k2Pi;
            }
            return radians;
        }

        void Rotation2d::calcRadians()
        {
            radians_ = std::atan2(sin_angle, cos_angle);
        }

        void Rotation2d::calcTrig()
        {
            cos_angle = std::cos(radians_);
            sin_angle = std::sin(radians_);
        }
    } // namespace team254_geometry
} // namespace ck