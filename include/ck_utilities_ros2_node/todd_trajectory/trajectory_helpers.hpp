#include "ck_utilities_ros2_node/geometry/geometry.hpp"

using geometry::Pose;

float calculate_desired_track (Pose starting_pose, Pose ending_pose);
float calculate_along_track_distance(Pose starting_pose, Pose ending_pose, float desired_track);
float calculate_track_angle_error(Pose starting_pose, Pose ending_pose, float desired_track);
float calculate_cross_track_distance(Pose starting_pose, Pose ending_pose, float desired_track);