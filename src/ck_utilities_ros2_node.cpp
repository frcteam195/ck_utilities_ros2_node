#include "ck_utilities_ros2_node/ck_utilities_ros2_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <thread>
#include <string>
#include <mutex>
#include <atomic>
#include <sys/stat.h>

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <array>

#define NODE_NAME "ck_utilities_ros2_node"

class LocalNode : public ParameterizedNode
{
public:
    LocalNode() : ParameterizedNode(NODE_NAME)
    {

    }

    ~LocalNode()
    {

    }

private:

};

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}