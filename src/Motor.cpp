#include <cstdint>
#if __has_include("ros/ros.h")
#include "ck_ros_base_msgs_node/Motor_Control.h"
#include "ck_ros_base_msgs_node/Motor_Configuration.h"
#include "ck_ros_base_msgs_node/Robot_Status.h"
#include "ck_utilities_ros2_node/Motor.hpp"

#include "ros/ros.h"
#include <mutex>
#include <map>
#include <thread>
#include <atomic>

extern ros::NodeHandle* node;

static std::recursive_mutex motor_mutex;

class MotorMaster
{
public:

    static void robot_status_callback(const ck_ros_base_msgs_node::Robot_Status& msg)
    {
        robot_mode = msg.robot_state;
    }

    static void store_motor_pointer(uint8_t id, Motor* motor)
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        motor_map[id] = motor;
    }

    static void create_motor_config(uint8_t id, Motor::Motor_Type type)
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        if (configuration_map.find(id) != configuration_map.end())
        {
            return;
        }
        configuration_map[id] = new MotorConfig();
        configuration_map[id]->motor_id = id;
        configuration_map[id]->active_config.motor_id = id;
        configuration_map[id]->pending_config.motor_id = id;
        configuration_map[id]->active_config.motor_config.id = id;
        configuration_map[id]->pending_config.motor_config.id = id;
        configuration_map[id]->active_config.motor_config.controller_type = (uint8_t) type;
        configuration_map[id]->pending_config.motor_config.controller_type = (uint8_t) type;
        configuration_map[id]->set_defaults();
        configuration_map[id]->apply();
    }

    static MotorConfig * retrieve_configuration(uint8_t id)
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        if(configuration_map.find(id) == configuration_map.end())
        {
            return nullptr;
        }
        return configuration_map[id];
    }

    static Motor * retrieve_motor(uint8_t id)
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        if(motor_map.find(id) == motor_map.end())
        {
            return nullptr;
        }
        return motor_map[id];
    }

    MotorMaster()
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        control_publisher = node->advertise<ck_ros_base_msgs_node::Motor_Control>("/MotorControl", 50);
        config_publisher = node->advertise<ck_ros_base_msgs_node::Motor_Configuration>("/MotorConfiguration", 50);
        robot_mode = 0;
        robot_data_subscriber = node->subscribe("RobotStatus", 10, robot_status_callback);


        motor_master_thread = new std::thread(motor_master_loop);
    }

    ~MotorMaster()
    {
        try
        {
            std::lock_guard<std::recursive_mutex> lock(motor_mutex);
            for(std::map<uint8_t, MotorConfig *>::iterator i = configuration_map.begin();
                i != configuration_map.end();
                i++)
            {
                delete (*i).second;
                (*i).second = nullptr;
            }
            for(std::map<uint8_t, Motor *>::iterator i = motor_map.begin();
                i != motor_map.end();
                i++)
            {
                delete (*i).second;
                (*i).second = nullptr;
            }
            configuration_map.clear();
            motor_map.clear();

            motor_master_thread->join();
        }
        catch ( ... ) { }

        try
        {
            motor_master_thread->join();
        }
        catch ( ... ) { }
    }

private:
    static std::thread * motor_master_thread;
    static ros::Publisher config_publisher;
    static ros::Publisher control_publisher;
    static ros::Subscriber robot_data_subscriber;
    static std::atomic<int8_t> robot_mode;
    static std::map<uint8_t, MotorConfig *> configuration_map;
    static std::map<uint8_t, Motor *> motor_map;

    static void send_motor_configs()
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);

        ck_ros_base_msgs_node::Motor_Configuration config_list;

        for(std::map<uint8_t, MotorConfig *>::iterator i = configuration_map.begin();
            i != configuration_map.end();
            i++)
        {
            config_list.motors.push_back((*i).second->active_config.motor_config);
        }

        config_publisher.publish(config_list);
    }

    static void send_master_controls_periodic()
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        static ck_ros_base_msgs_node::Motor_Control motor_control_list;
        motor_control_list.motors.clear();

        for(std::map<uint8_t, Motor *>::iterator i = motor_map.begin();
            i != motor_map.end();
            i++)
        {
            Motor* m = (*i).second;
            if (m->mValueLock.try_lock())
            {
                if(m->config().active_config.motor_config.controller_mode == ck_ros_base_msgs_node::Motor_Config::MASTER ||
                m->config().active_config.motor_config.controller_mode == ck_ros_base_msgs_node::Motor_Config::FAST_MASTER)
                {
                    ck_ros_base_msgs_node::Motor motor;
                    motor.controller_type = m->config().active_config.motor_config.controller_type;
                    motor.id = m->id;
                    Motor::Control_Mode tmpCtrl = m->mControlMode;
                    motor.control_mode = (int8_t)tmpCtrl;
                    motor.output_value = m->mOutput;
                    motor.arbitrary_feedforward = m->mArbFF;

                    motor_control_list.motors.push_back(motor);
                }
                m->mValueLock.unlock();
            }
        }

        control_publisher.publish(motor_control_list);
    }

    static void send_follower_controls()
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);

        ck_ros_base_msgs_node::Motor_Control motor_control_list;

        for(std::map<uint8_t, MotorConfig *>::iterator i = configuration_map.begin();
            i != configuration_map.end();
            i++)
        {
            if((*i).second->active_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::FOLLOW_MASTER ||
               (*i).second->active_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::OPPOSE_MASTER)
            {
                ck_ros_base_msgs_node::Motor motor;
                motor.controller_type = (*i).second->active_config.motor_config.controller_type;
                motor.id = (*i).second->active_config.motor_config.id;
                motor.control_mode = ck_ros_base_msgs_node::Motor::FOLLOWER;
                motor.output_value = (*i).second->active_config.master_id;
                motor_control_list.motors.push_back(motor);
            }
        }

        control_publisher.publish(motor_control_list);
    }

    static void motor_master_loop()
    {
        ros::Rate timer(10);
        while(ros::ok())
        {
            send_motor_configs();
            send_master_controls_periodic();
            send_follower_controls();
            timer.sleep();
        }
    }

friend class Motor;
friend class MotorConfig;
};

std::map<uint8_t, MotorConfig *> MotorMaster::configuration_map;
std::map<uint8_t, Motor *> MotorMaster::motor_map;
std::thread * MotorMaster::motor_master_thread;
ros::Publisher MotorMaster::config_publisher;
ros::Publisher MotorMaster::control_publisher;
ros::Subscriber MotorMaster::robot_data_subscriber;
std::atomic<int8_t> MotorMaster::robot_mode;

static MotorMaster * motor_master = nullptr;

void MotorConfig::apply()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->active_config = this->pending_config;
}

void MotorConfig::set_fast_master(bool enable)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if(enable)
    {
        this->pending_config.motor_config.controller_mode = ck_ros_base_msgs_node::Motor_Config::FAST_MASTER;
    }
    else
    {
        this->pending_config.motor_config.controller_mode = ck_ros_base_msgs_node::Motor_Config::MASTER;
    }
}

void MotorConfig::set_kP(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kP = value;
}

void MotorConfig::set_kI(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kI = value;
}

void MotorConfig::set_kD(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kD = value;
}

void MotorConfig::set_kF(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kF = value;
}

void MotorConfig::set_kP_Slot1(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kP_1 = value;
}

void MotorConfig::set_kI_Slot1(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kI_1 = value;
}

void MotorConfig::set_kD_Slot1(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kD_1 = value;
}

void MotorConfig::set_kF_Slot1(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.kF_1 = value;
}

void MotorConfig::set_active_gain_slot(int8_t slotIdx)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.active_gain_slot = slotIdx;
}

void MotorConfig::set_i_zone(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.iZone = value;
}

void MotorConfig::set_max_i_accum(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.max_i_accum = value;
}

void MotorConfig::set_allowed_closed_loop_error(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.allowed_closed_loop_error = value;
}

void MotorConfig::set_max_closed_loop_peak_output(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.max_closed_loop_peak_output = value;
}

void MotorConfig::set_motion_cruise_velocity(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.motion_cruise_velocity = value;
}

void MotorConfig::set_motion_acceleration(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.motion_acceleration = value;
}

void MotorConfig::set_motion_s_curve_strength(int32_t value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.motion_s_curve_strength = value;
}

void MotorConfig::set_forward_soft_limit(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.forward_soft_limit = value;
}

void MotorConfig::set_forward_soft_limit_enable(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.forward_soft_limit_enable = enabled;
}

void MotorConfig::set_reverse_soft_limit(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.reverse_soft_limit = value;
}

void MotorConfig::set_reverse_soft_limit_enable(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.reverse_soft_limit_enable = enabled;
}

void MotorConfig::set_feedback_sensor_coefficient(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.feedback_sensor_coefficient = value;
}

void MotorConfig::set_voltage_compensation_saturation(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.voltage_compensation_saturation = value;
}

void MotorConfig::set_voltage_compensation_enabled(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.voltage_compensation_enabled = enabled;
}

void MotorConfig::set_inverted(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if(this->pending_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::FOLLOW_MASTER ||
       this->pending_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::OPPOSE_MASTER)
    {
        this->pending_config.motor_config.invert_type = enabled ? ck_ros_base_msgs_node::Motor_Config::OPPOSE_MASTER : ck_ros_base_msgs_node::Motor_Config::FOLLOW_MASTER;
        return;
    }
    this->pending_config.motor_config.invert_type = enabled ? ck_ros_base_msgs_node::Motor_Config::INVERT_MOTOR_OUTPUT : ck_ros_base_msgs_node::Motor_Config::NONE;

}

void MotorConfig::set_sensor_phase_inverted(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.sensor_phase_inverted = enabled;
}

void MotorConfig::set_neutral_mode(MotorConfig::NeutralMode mode)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.neutral_mode = (uint8_t) mode;
}

void MotorConfig::set_open_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.open_loop_ramp = value;
}

void MotorConfig::set_closed_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.closed_loop_ramp = value;
}

void MotorConfig::set_supply_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.supply_current_limit_config.enable = enabled;
    this->pending_config.motor_config.supply_current_limit_config.current_limit = current_limit;
    this->pending_config.motor_config.supply_current_limit_config.trigger_threshold_current = trigger_current;
    this->pending_config.motor_config.supply_current_limit_config.trigger_threshold_time = trigger_time;
}

void MotorConfig::set_stator_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.stator_current_limit_config.enable = enabled;
    this->pending_config.motor_config.stator_current_limit_config.current_limit = current_limit;
    this->pending_config.motor_config.stator_current_limit_config.trigger_threshold_current = trigger_current;
    this->pending_config.motor_config.stator_current_limit_config.trigger_threshold_time = trigger_time;
}

void MotorConfig::set_follower(bool enabled, uint8_t master_id)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if(enabled)
    {
        this->pending_config.master_id = master_id;
        if(this->pending_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::OPPOSE_MASTER ||
        this->pending_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::INVERT_MOTOR_OUTPUT)
        {
            this->pending_config.motor_config.invert_type = ck_ros_base_msgs_node::Motor_Config::OPPOSE_MASTER;
            return;
        }
        this->pending_config.motor_config.invert_type = ck_ros_base_msgs_node::Motor_Config::FOLLOW_MASTER;
        this->pending_config.motor_config.controller_mode = ck_ros_base_msgs_node::Motor_Config::FOLLOWER;
        return;
    }
    this->pending_config.master_id = 0;
    this->pending_config.motor_config.controller_mode = ck_ros_base_msgs_node::Motor_Config::MASTER;
    if(this->pending_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::OPPOSE_MASTER ||
       this->pending_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::INVERT_MOTOR_OUTPUT)
    {
        this->pending_config.motor_config.invert_type = ck_ros_base_msgs_node::Motor_Config::INVERT_MOTOR_OUTPUT;
        return;
    }
    this->pending_config.motor_config.invert_type = ck_ros_base_msgs_node::Motor_Config::NONE;
}

void MotorConfig::set_forward_limit_switch(LimitSwitchSource forward_limit_switch_source, LimitSwitchNormal forward_limit_switch_normal)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.forward_limit_switch_source = (int8_t)forward_limit_switch_source;
    this->pending_config.motor_config.forward_limit_switch_normal = (int8_t)forward_limit_switch_normal;
}

void MotorConfig::set_reverse_limit_switch(LimitSwitchSource reverse_limit_switch_source, LimitSwitchNormal reverse_limit_switch_normal)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.reverse_limit_switch_source = (int8_t)reverse_limit_switch_source;
    this->pending_config.motor_config.reverse_limit_switch_normal = (int8_t)reverse_limit_switch_normal;
}

void MotorConfig::set_peak_output_forward(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.peak_output_forward = value;
}

void MotorConfig::set_peak_output_reverse(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.peak_output_reverse = value;
}

void MotorConfig::set_defaults()
{
    this->set_fast_master(false);
    this->set_kP(0.0);
    this->set_kI(0.0);
    this->set_kD(0.0);
    this->set_kF(0.0);
    this->set_kP_Slot1(0.0);
    this->set_kI_Slot1(0.0);
    this->set_kD_Slot1(0.0);
    this->set_kF_Slot1(0.0);
    this->set_active_gain_slot(0);
    this->set_i_zone(0.0);
    this->set_max_i_accum(0.0);
    this->set_allowed_closed_loop_error(0.0);
    this->set_max_closed_loop_peak_output(0.0);
    this->set_motion_cruise_velocity(0.0);
    this->set_motion_acceleration(0.0);
    this->set_motion_s_curve_strength(0);
    this->set_forward_soft_limit(0.0);
    this->set_forward_soft_limit_enable(false);
    this->set_reverse_soft_limit(0.0);
    this->set_reverse_soft_limit_enable(false);
    this->set_feedback_sensor_coefficient(0.0);
    this->set_voltage_compensation_saturation(12.0);
    this->set_voltage_compensation_enabled(true);
    this->set_inverted(false);
    this->set_sensor_phase_inverted(false);
    this->set_neutral_mode(MotorConfig::NeutralMode::COAST);
    this->set_open_loop_ramp(0.0);
    this->set_closed_loop_ramp(0.0);
    this->set_supply_current_limit(true, 40.0, 0.0, 0.0);
    this->set_stator_current_limit(false, 0.0, 0.0, 0.0);
    this->set_follower(false, 0);
    this->set_forward_limit_switch(LimitSwitchSource::Deactivated, LimitSwitchNormal::Disabled);
    this->set_reverse_limit_switch(LimitSwitchSource::Deactivated, LimitSwitchNormal::Disabled);
    this->set_peak_output_forward(0);
    this->set_peak_output_reverse(0);
}

Motor::Motor(uint8_t id, Motor_Type type)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if(motor_master == nullptr)
    {
        motor_master = new MotorMaster();
    }
    this->id = id;
    motor_master->create_motor_config(id, type);
    motor_master->store_motor_pointer(id, this);
}

void Motor::set(Control_Mode mode, double output, double arbitrary_feedforward)
{
    std::lock_guard<std::recursive_mutex> lock(mValueLock);
    mControlMode = mode;
    mOutput = output;
    mArbFF = arbitrary_feedforward;

    ck_ros_base_msgs_node::Motor_Control motors;
    ck_ros_base_msgs_node::Motor motor;
    motor.controller_type = this->config().active_config.motor_config.controller_type;
    motor.id = id;
    motor.control_mode = (uint8_t) mode;
    motor.output_value = output;
    motor.arbitrary_feedforward = arbitrary_feedforward;
    motors.motors.push_back(motor);
    static ros::Publisher motor_control_pub = node->advertise<ck_ros_base_msgs_node::Motor_Control>("MotorControl", 50);
    motor_control_pub.publish(motors);
}

MotorConfig& Motor::config()
{
    return *(motor_master->retrieve_configuration(this->id));
}

#endif