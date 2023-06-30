#include "ck_utilities_ros2_node/PIDTuner.hpp"
#include "ck_utilities_ros2_node/node_handle.hpp"

namespace ck
{
    PIDTuner::PIDTuner(std::string topic_basename, PIDController *pid)
    {
        this->topic_basename = topic_basename;
        this->pid = pid;

        std::string topic_name = "/" + topic_basename + "Gains";

        actual_gains_pub = node_handle->create_publisher<ck_ros2_base_msgs_node::msg::PIDTuning>(topic_name + "Actual", 10);

        set_gains_sub = node_handle->create_subscription<ck_ros2_base_msgs_node::msg::PIDTuning>(
            std::string(topic_name + "Set"),
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(),
            std::bind(&PIDTuner::set_gains_callback, this, std::placeholders::_1)
            );

        pub_thread = std::thread(&PIDTuner::update, this);
    }

    PIDTuner::~PIDTuner()
    {
        pub_thread.join();
    }

    void PIDTuner::update()
    {
        rclcpp::Rate rate(100);

        while (rclcpp::ok())
        {
            ck_ros2_base_msgs_node::msg::PIDTuning tuning;
            tuning.k_p = pid->kP;
            tuning.k_i = pid->kI;
            tuning.k_d = pid->kD;
            tuning.feed_forward = pid->feedforward;
            tuning.filter_r = pid->filter_r;
            tuning.dt = pid->dt;
            tuning.setpoint = pid->setpoint_overrride;
            tuning.actual = pid->actual;

            actual_gains_pub->publish(tuning);

            rate.sleep();
        }

        RCLCPP_ERROR(node_handle->get_logger(), "ROS NOT OK, QUIT UPDATING PID");
    }

    void PIDTuner::set_gains_callback(const ck_ros2_base_msgs_node::msg::PIDTuning &tuning)
    {
        RCLCPP_ERROR(node_handle->get_logger(), "Got gains p=%0.2f, i=%0.2f, d=%0.2f, filter_r=%0.2f", tuning.k_p, tuning.k_i, tuning.k_d, tuning.filter_r);
        pid->setGains(tuning.k_p, tuning.k_i, tuning.k_d, tuning.feed_forward);
        pid->setFilter(tuning.filter_r);
        pid->setSetpointOverride(tuning.setpoint);
    }
} // namespace ck
