#pragma once

#include <ck_ros2_base_msgs_node/msg/joystick_status.hpp>
#include <ck_ros2_base_msgs_node/msg/joystick_status_array.hpp>
#include <thread>
#include <map>
#include <mutex>
#include <sstream>

#define MAX_NUM_JOYSTICKS 6
#define MAX_NUM_AXES 8
#define MAX_NUM_BUTTONS 16
#define MAX_NUM_POVS 4

class JoystickIDOutOfRangeException: public std::exception
{
private:
    uint mId;

public:
    JoystickIDOutOfRangeException(uint id)
    {
        mId = id;
    }

    virtual const char* what() const throw()
    {
        std::stringstream ss;
        ss << "Joystick ID " << mId << " out of range!" << std::endl;
        return ss.str().c_str();
    }

};

class Joystick
{
public:
    Joystick(uint joystickID);
    static void update(const ck_ros2_base_msgs_node::msg::JoystickStatusArray& joystick_status_msg);
    double getRawAxis(uint axisID);
    double getFilteredAxis(uint axisID, double deadband);
    bool getAxisActuated(uint axisID, float threshold);
    bool getButton(uint buttonID);
    bool getRisingEdgeButton(uint buttonID);
    bool getFallingEdgeButton(uint buttonID);
    int getPOV(uint povID);
private:
    static ck_ros2_base_msgs_node::msg::JoystickStatusArray joystick_status;
    static std::map<int, ck_ros2_base_msgs_node::msg::JoystickStatus>  joystick_map;
    bool mPrevButtonValues[MAX_NUM_BUTTONS] = {0};
    int mPrevPOV = 0;
    int mJoystickID;
};