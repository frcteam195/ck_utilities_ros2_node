#include "ck_utilities_ros2_node/Logger.hpp"
// #if __has_include("rclcpp/rclcpp.hpp")

#include "rclcpp/rclcpp.hpp"
#include "ck_utilities_ros2_node/node_handle.hpp"


#include <sstream>

enum class ros_log_level
{
    DEBUG,
    INFO,
    WARN,
    ERROR,
    FATAL,
};

class logger : public std::stringbuf
{
public:
    logger(ros_log_level level)
    {
        this->log_level = level;
    }

    virtual int sync()
    {
        switch(this->log_level)
        {
            case ros_log_level::DEBUG:
            {
                RCLCPP_DEBUG(node_handle->get_logger(), "%s", this->str().c_str());
                break;
            }
            case ros_log_level::INFO:
            {
                RCLCPP_INFO(node_handle->get_logger(), "%s", this->str().c_str());
                break;
            }
            case ros_log_level::WARN:
            {
                RCLCPP_WARN(node_handle->get_logger(), "%s", this->str().c_str());
                break;
            }
            case ros_log_level::ERROR:
            {
                RCLCPP_ERROR(node_handle->get_logger(), "%s", this->str().c_str());
                break;
            }
            case ros_log_level::FATAL:
            {
                RCLCPP_FATAL(node_handle->get_logger(), "%s", this->str().c_str());
                break;
            }
        }
        this->str("");
        return 0;
    }

private:
    ros_log_level log_level;
};

logger debug_buf(ros_log_level::DEBUG);
logger info_buf(ros_log_level::INFO);
logger warn_buf(ros_log_level::WARN);
logger error_buf(ros_log_level::ERROR);
logger fatal_buf(ros_log_level::FATAL);

std::ostream ck::log_debug(&debug_buf);
std::ostream ck::log_info(&info_buf);
std::ostream ck::log_warn(&warn_buf);
std::ostream ck::log_error(&error_buf);
std::ostream ck::log_fatal(&fatal_buf);
// #endif