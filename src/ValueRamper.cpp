// #include "ck_utilities_ros2_node/ValueRamper.hpp"
// #if __has_include("ros/ros.h")
// #include "ros/ros.h"

// ValueRamper::ValueRamper(double accelRampRate, double decelRampRate, double zeroValue, double maxValue)
// : mAccelRampRate(accelRampRate), mDecelRampRate(decelRampRate), mZeroValue(zeroValue), mMaxValue(maxValue)
// {}

// void ValueRamper::update_params(double accelRampRate, double decelRampRate, double zeroValue, double maxValue)
// {
//     mAccelRampRate = accelRampRate;
//     mDecelRampRate = decelRampRate;
//     mZeroValue = zeroValue;
//     mMaxValue = maxValue;
// }

// void ValueRamper::reset()
// {
//     mPrevTime = ros::Time::now();
//     mPrevValue = mZeroValue;
// }

// double ValueRamper::get_value()
// {
//     return mPrevValue;
// }

// double ValueRamper::calculateOutput(double currValue)
// {
//     ros::Time timeNow = ros::Time::now();
//     if (mPrevTime != ros::Time(0))
//     {
//         double dt = (timeNow - mPrevTime).toSec();
//         double accelStep = (mMaxValue - mZeroValue) * dt / mAccelRampRate;
//         double decelStep = (mMaxValue - mZeroValue) * dt / mDecelRampRate;
//         mPrevTime = timeNow;
//         // std::stringstream o;
//         // o << "DT: " << dt << " accel " << accelStep << " decel " << decelStep << " prev " << mPrevValue << " curr " << currValue;
//         // ROS_INFO("%s", o.str().c_str());
//         if (currValue > mPrevValue && currValue >= 0)
//         {
//             //accelerating
//             mPrevValue += accelStep;
//             mPrevValue = std::min(mPrevValue, currValue);
//             return mPrevValue;
//         }
//         else if(currValue < mPrevValue && currValue >= 0)
//         {
//             //decelerating
//             mPrevValue -= decelStep;
//             mPrevValue = std::max(mPrevValue, currValue);
//             return mPrevValue;
//         }
//         else if (currValue < mPrevValue && currValue <= 0)
//         {
//             //accelerating
//             mPrevValue -= accelStep;
//             mPrevValue = std::max(mPrevValue, currValue);
//             return mPrevValue;
//         }
//         else if (currValue > mPrevValue && currValue <= 0)
//         {
//             //decelerating
//             mPrevValue += decelStep;
//             mPrevValue = std::min(mPrevValue, currValue);
//             return mPrevValue;
//         }
//         else
//         {
//             mPrevValue = currValue;
//             return currValue;
//         }
//     }
//     mPrevTime = timeNow;
//     return 0;
// }
// #endif