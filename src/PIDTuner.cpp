// #include "ck_utilities_ros2_node/PIDTuner.hpp"

// namespace ck
// {
//     PIDTuner::PIDTuner(ros::NodeHandle *n, std::string topic_basename, PIDController *pid)
//     {
//         this->n = n;
//         this->topic_basename = topic_basename;
//         this->pid = pid;

//         std::string topic_name = "/" + topic_basename + "Gains";

//         actual_gains_pub = n->advertise<ck_ros_base_msgs_node::PID_Tuning>(
//             topic_name + "Actual",
//             10);

//         set_gains_sub = n->subscribe(
//             std::string(topic_name + "Set"),
//             10,
//             &PIDTuner::set_gains_callback,
//             this,
//             ros::TransportHints().tcpNoDelay());

//         pub_thread = std::thread(&PIDTuner::update, this);
//     }

//     PIDTuner::~PIDTuner()
//     {
//         pub_thread.join();
//     }

//     void PIDTuner::update()
//     {
//         ros::Rate rate(100);

//         while (ros::ok())
//         {
//             ck_ros_base_msgs_node::PID_Tuning tuning;
//             tuning.kP = pid->kP;
//             tuning.kI = pid->kI;
//             tuning.kD = pid->kD;
//             tuning.feedforward = pid->feedforward;
//             tuning.filter_r = pid->filter_r;
//             tuning.dt = pid->dt;
//             tuning.setpoint = pid->setpoint_overrride;
//             tuning.actual = pid->actual;

//             actual_gains_pub.publish(tuning);

//             rate.sleep();
//         }

//         ROS_ERROR("ROS NOT OK, QUIT UPDATING PID");
//     }

//     void PIDTuner::set_gains_callback(const ck_ros_base_msgs_node::PID_Tuning &tuning)
//     {
//         ROS_ERROR("Got gains p=%0.2f, i=%0.2f, d=%0.2f, filter_r=%0.2f", tuning.kP, tuning.kI, tuning.kD, tuning.filter_r);
//         pid->setGains(tuning.kP, tuning.kI, tuning.kD, tuning.feedforward);
//         pid->setFilter(tuning.filter_r);
//         pid->setSetpointOverride(tuning.setpoint);
//     }
// } // namespace ck
