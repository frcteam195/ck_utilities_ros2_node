#include "ck_utilities_ros2_node/Joystick.hpp"
#include "ck_utilities_ros2_node/CKMath.hpp"

ck_ros2_base_msgs_node::msg::JoystickStatusArray Joystick::joystick_status;
std::map<int, ck_ros2_base_msgs_node::msg::JoystickStatus> Joystick::joystick_map;

Joystick::Joystick(uint joystickID)
: mJoystickID(joystickID)
{
    if (joystickID >= MAX_NUM_JOYSTICKS)
    {
        throw JoystickIDOutOfRangeException(joystickID);
    }
}

void Joystick::update(const ck_ros2_base_msgs_node::msg::JoystickStatusArray& joystick_status_msg)
{
    joystick_status = joystick_status_msg;
    joystick_map.clear();
    for (const ck_ros2_base_msgs_node::msg::JoystickStatus& j : joystick_status_msg.joysticks)
    {
        joystick_map.emplace(j.index, j);
    }
}

double Joystick::getRawAxis(uint axisID)
{
    if (joystick_map.count(mJoystickID))
    {
        if (axisID < MAX_NUM_AXES && joystick_map[mJoystickID].axes.size() > axisID)
        {
            return joystick_status.joysticks[mJoystickID].axes[axisID];
        }
    }
    return 0;
}

double Joystick::getFilteredAxis(uint axisID, double deadband)
{
    return ck::math::normalizeWithDeadband(getRawAxis(axisID), deadband);
}

bool Joystick::getAxisActuated(uint axisID, float threshold)
{
    return getRawAxis(axisID) > threshold;
}

bool Joystick::getButton(uint buttonID)
{
    if (buttonID < MAX_NUM_BUTTONS)
    {
        bool retVal = false;
        if (joystick_map.count(mJoystickID))
        {
            if (joystick_map[mJoystickID].buttons.size() > buttonID)
            {
                retVal = joystick_map[mJoystickID].buttons[buttonID];
            }
        }

        mPrevButtonValues[buttonID] = retVal;
        return retVal;
    }
    return false;
}

bool Joystick::getRisingEdgeButton(uint buttonID)
{
    if (buttonID < MAX_NUM_BUTTONS)
    {
        bool currVal = false;
        if (joystick_map.count(mJoystickID))
        {
            if (joystick_map[mJoystickID].buttons.size() > buttonID)
            {
                currVal = joystick_map[mJoystickID].buttons[buttonID];
            }
        }

        bool retVal = currVal && (currVal != mPrevButtonValues[buttonID]);
        mPrevButtonValues[buttonID] = currVal;
        return retVal;
    }
    return false;
}

bool Joystick::getFallingEdgeButton(uint buttonID)
{
    if (buttonID < MAX_NUM_BUTTONS)
    {
        bool currVal = false;
        if (joystick_map.count(mJoystickID))
        {
            if (joystick_map[mJoystickID].buttons.size() > buttonID)
            {
                currVal = joystick_map[mJoystickID].buttons[buttonID];
            }
        }

        bool retVal = !currVal && (currVal != mPrevButtonValues[buttonID]);
        mPrevButtonValues[buttonID] = currVal;
        return retVal;
    }
    return false;
}

int Joystick::getPOV(uint povID)
{
    if (joystick_map.count(mJoystickID))
    {
        if (povID < MAX_NUM_POVS && joystick_map[mJoystickID].povs.size() > povID)
        {
            return joystick_status.joysticks[mJoystickID].povs[povID];
        }
    }
    return -1;
}