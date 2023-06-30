#pragma once

#include "ck_utilities_ros2_node/PIDController.hpp"

#include "ck_ros2_base_msgs_node/msg/pid_tuning.hpp"

#include "rclcpp/rclcpp.hpp"

#include <string>
#include <thread>

namespace ck
{
    class PIDController;

    class PIDTuner
    {
    public:
        PIDTuner(std::string topic_basename, PIDController *pid);
        ~PIDTuner();

        void update();

    private:
        void set_gains_callback(const ck_ros2_base_msgs_node::msg::PIDTuning &tuning);

        std::string topic_basename;
        PIDController *pid;

        rclcpp::Publisher<ck_ros2_base_msgs_node::msg::PIDTuning>::SharedPtr actual_gains_pub;
        rclcpp::Publisher<ck_ros2_base_msgs_node::msg::PIDTuning>::SharedPtr set_gains_pub;
        rclcpp::Subscription<ck_ros2_base_msgs_node::msg::PIDTuning>::SharedPtr set_gains_sub;

        std::thread pub_thread;
    };
} // namespace ck
