#pragma once
#include <cstdint>
#include "ck_ros2_base_msgs_node/msg/motor_control.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_configuration.hpp"
#include <atomic>
#include <mutex>

class MotorMaster;

class MotorData
{
public:
    uint8_t motor_id;
    ck_ros2_base_msgs_node::msg::MotorConfiguration motor_config;
    uint8_t master_id;
};

class MotorConfig
{
private:
    uint8_t motor_id;
    MotorData active_config;
    MotorData pending_config;
public:

    enum class NeutralMode
    {
        COAST=1,
        BRAKE=2,
    };

    void apply();
    void set_kP(double value, uint8_t slot);
    void set_kI(double value, uint8_t slot);
    void set_kD(double value, uint8_t slot);
    void set_kV(double value, uint8_t slot);
    void set_kS(double value, uint8_t slot);
    void set_motion_magic_cruise_velocity(double value);
    void set_motion_magic_acceleration(double value);
    void set_motion_magic_jerk(double value);
    void set_forward_soft_limit(double value);
    void set_forward_soft_limit_enable(bool enabled);
    void set_reverse_soft_limit(double value);
    void set_reverse_soft_limit_enable(bool enabled);
    void set_inverted(bool enabled);
    void set_neutral_mode(NeutralMode mode);
    void set_duty_cycle_open_loop_ramp(double value);
    void set_torque_current_open_loop_ramp(double value);
    void set_voltage_open_loop_ramp(double value);
    void set_duty_cycle_closed_loop_ramp(double value);
    void set_torque_current_closed_loop_ramp(double value);
    void set_voltage_closed_loop_ramp(double value);
    void set_supply_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time);
    void set_stator_current_limit(bool enabled, double current_limit);
    void set_follower(uint8_t master_id);
    void set_defaults();

protected:
    MotorConfig() { };

friend class MotorMaster;
friend class Motor;
};

class Motor
{
public:
    enum class ControlMode : int
    {
        DUTY_CYCLE = 0,
        TORQUE_CURRENT = 1,
        VOLTAGE = 2,
        POSITION = 3,
        VELOCITY = 4,
        MOTION_MAGIC = 5,
        NEUTRAL_OUT = 6,
        STATIC_BRAKE = 7,
        COAST_OUT = 8,
        FOLLOWER = 9,
    };

    enum class FeedForwardType : int
    {
        NONE = 0,
        DUTY_CYCLE = 1,
        TORQUE_CURRENT = 2,
        VOLTAGE = 3
    };

    Motor(uint8_t id);
    void set(ControlMode mode, double setpoint, FeedForwardType feed_forward_type, double feed_forward, uint8_t gain_slot = 0);
    MotorConfig& config();

private:
    Motor() = delete;
    uint8_t id;

    std::recursive_mutex mValueLock;
    ControlMode mControlMode {ControlMode::DUTY_CYCLE};
    double mOutput = 0;
    double mArbFF = 0;
    FeedForwardType mFFType {FeedForwardType::NONE};
    uint8_t mGainSlot;

friend class MotorMaster;
};