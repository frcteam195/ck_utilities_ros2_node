#include "ck_utilities_ros2_node/geometry/geometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_eigen/tf2_eigen.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "boost/array.hpp"

namespace geometry
{
    geometry::Rotation to_rotation(tf2::Quaternion q);
    geometry::Rotation to_rotation(geometry_msgs::msg::Quaternion q);
    geometry::Rotation to_rotation(tf2::Vector3 v);
    geometry::Rotation to_rotation(geometry_msgs::msg::Vector3 v);
    geometry::Translation to_translation(tf2::Vector3 v);
    geometry::Translation to_translation(geometry_msgs::msg::Vector3 v);
    geometry::Translation to_translation(geometry_msgs::msg::Point p);
    geometry::Pose to_pose(geometry_msgs::msg::Pose p);
    geometry::Transform to_transform(geometry_msgs::msg::Transform t);
    geometry::Twist to_twist(geometry_msgs::msg::Twist t);
    geometry::Covariance to_covariance(boost::array<double, 36UL> &c);
    tf2::Quaternion to_tf2_quat(geometry::Rotation r);
    geometry_msgs::msg::Quaternion to_msg_quat(geometry::Rotation r);
    tf2::Vector3 to_tf2(geometry::Translation t);
    tf2::Transform to_tf2(geometry::Transform t);
    tf2::Transform to_tf2(geometry::Transform t);
    geometry_msgs::msg::Transform to_msg(geometry::Transform t);
    geometry_msgs::msg::Point to_msg_point(geometry::Translation t);
    tf2::Vector3 to_tf2(geometry::Rotation r);
    geometry_msgs::msg::Vector3 to_msg(geometry::Rotation r);
    geometry_msgs::msg::Vector3 to_msg(geometry::Translation t);
    geometry_msgs::msg::Pose to_msg(geometry::Pose p);
    geometry_msgs::msg::Twist to_msg(geometry::Twist t);
    geometry_msgs::msg::Transform to_msg(geometry::Transform t);
    std::array<double, 36UL> to_msg(geometry::Covariance &c);
}