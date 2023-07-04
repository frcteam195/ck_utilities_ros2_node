#include <cstdint>
#include <cstdio>
#include <iostream>
#include "ck_ros2_base_msgs_node/msg/motor_control.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_control_array.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_configuration.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_configuration_array.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_status.hpp"
#include "ck_ros2_base_msgs_node/msg/motor_status_array.hpp"
#include "ck_ros2_base_msgs_node/msg/robot_status.hpp"
#include "ck_utilities_ros2_node/Motor.hpp"
#include "ck_utilities_ros2_node/CKMath.hpp"

#include "rclcpp/rclcpp.hpp"
#include <mutex>
#include <map>
#include <thread>
#include <atomic>
#include "ck_utilities_ros2_node/node_handle.hpp"

static std::recursive_mutex motor_mutex;

class MotorMaster
{
public:

    static void robot_status_callback(const ck_ros2_base_msgs_node::msg::RobotStatus::SharedPtr msg)
    {
        robot_mode = msg->robot_state;
    }

    static void motor_status_callback(const ck_ros2_base_msgs_node::msg::MotorStatusArray::SharedPtr msg)
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        for (auto m : msg->motors)
        {
            if(status_map.find(m.id) == status_map.end())
            {
                status_map[m.id] = new MotorStatus();
            }
            status_map[m.id]->motor_id = m.id;
            status_map[m.id]->status_data = m;
        }
    }

    static void store_motor_pointer(uint8_t id, Motor* motor)
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        motor_map[id] = motor;
    }

    static void create_motor_config(uint8_t id)
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
        configuration_map[id]->pending_config.motor_config.k_p = std::vector<double>{0,0,0};
        configuration_map[id]->pending_config.motor_config.k_i = std::vector<double>{0,0,0};
        configuration_map[id]->pending_config.motor_config.k_d = std::vector<double>{0,0,0};
        configuration_map[id]->pending_config.motor_config.k_v = std::vector<double>{0,0,0};
        configuration_map[id]->pending_config.motor_config.k_s = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_p = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_i = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_d = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_v = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_s = std::vector<double>{0,0,0};
        configuration_map[id]->set_defaults();
        configuration_map[id]->apply();
    }

        static void create_motor_config_from_params(uint8_t id, MotorConfigurationParameters params)
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
        configuration_map[id]->pending_config.motor_config.k_p = std::vector<double>{0,0,0};
        configuration_map[id]->pending_config.motor_config.k_i = std::vector<double>{0,0,0};
        configuration_map[id]->pending_config.motor_config.k_d = std::vector<double>{0,0,0};
        configuration_map[id]->pending_config.motor_config.k_v = std::vector<double>{0,0,0};
        configuration_map[id]->pending_config.motor_config.k_s = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_p = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_i = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_d = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_v = std::vector<double>{0,0,0};
        configuration_map[id]->active_config.motor_config.k_s = std::vector<double>{0,0,0};
        configuration_map[id]->load_parameters(params);
        configuration_map[id]->apply();
    }

    static MotorStatus * retrieve_status(uint8_t id)
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        if(status_map.find(id) == status_map.end())
        {
            return nullptr;
        }
        return status_map[id];
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
        control_publisher = node_handle->create_publisher<ck_ros2_base_msgs_node::msg::MotorControlArray>("/MotorControl", 50);
        config_publisher = node_handle->create_publisher<ck_ros2_base_msgs_node::msg::MotorConfigurationArray>("/MotorConfiguration", 50);
        robot_mode = 0;
        robot_data_subscriber = node_handle->create_subscription<ck_ros2_base_msgs_node::msg::RobotStatus>("RobotStatus", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), MotorMaster::robot_status_callback);

        motor_status_subscriber = node_handle->create_subscription<ck_ros2_base_msgs_node::msg::MotorStatusArray>("MotorStatus", rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile(), MotorMaster::motor_status_callback);

        motor_master_thread = new std::thread(motor_master_loop);
    }

    ~MotorMaster()
    {
        try
        {
            std::lock_guard<std::recursive_mutex> lock(motor_mutex);
            for(std::map<uint8_t, MotorStatus *>::iterator i = status_map.begin();
                i != status_map.end();
                i++)
            {
                delete (*i).second;
                (*i).second = nullptr;
            }
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
            status_map.clear();
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
    static rclcpp::Publisher<ck_ros2_base_msgs_node::msg::MotorConfigurationArray>::SharedPtr config_publisher;
    static rclcpp::Publisher<ck_ros2_base_msgs_node::msg::MotorControlArray>::SharedPtr control_publisher;
    static rclcpp::Subscription<ck_ros2_base_msgs_node::msg::MotorStatusArray>::SharedPtr motor_status_subscriber;
    static rclcpp::Subscription<ck_ros2_base_msgs_node::msg::RobotStatus>::SharedPtr robot_data_subscriber;
    static std::atomic<int8_t> robot_mode;
    static std::map<uint8_t, MotorStatus *> status_map;
    static std::map<uint8_t, MotorConfig *> configuration_map;
    static std::map<uint8_t, Motor *> motor_map;

    static void send_motor_configs()
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);

        ck_ros2_base_msgs_node::msg::MotorConfigurationArray config_list;

        for(std::map<uint8_t, MotorConfig *>::iterator i = configuration_map.begin();
            i != configuration_map.end();
            i++)
        {
            config_list.motors.push_back((*i).second->active_config.motor_config);
        }

        config_publisher->publish(config_list);
    }

    static void send_master_controls_periodic()
    {
        std::lock_guard<std::recursive_mutex> lock(motor_mutex);
        static ck_ros2_base_msgs_node::msg::MotorControlArray motor_control_list;
        motor_control_list.motors.clear();

        for(std::map<uint8_t, Motor *>::iterator i = motor_map.begin();
            i != motor_map.end();
            i++)
        {
            Motor* m = (*i).second;
            if (m->mValueLock.try_lock())
            {
                if(m->mControlMode != Motor::ControlMode::FOLLOWER)
                {
                    ck_ros2_base_msgs_node::msg::MotorControl motor;
                    motor.id = m->id;
                    motor.control_mode = (uint8_t)(m->mControlMode);
                    motor.setpoint = m->mOutput;
                    motor.feed_forward_type = (uint8_t)(m->mFFType);
                    motor.feed_forward = m->mArbFF;
                    motor.gain_slot = m->mGainSlot;

                    motor_control_list.motors.push_back(motor);
                }
                m->mValueLock.unlock();
            }
        }

        control_publisher->publish(motor_control_list);
    }

    static void send_follower_controls()
    {
        // std::lock_guard<std::recursive_mutex> lock(motor_mutex);

        // ck_ros2_base_msgs_node::msg::MotorControlArray motor_control_list;

        // for(std::map<uint8_t, MotorConfig *>::iterator i = configuration_map.begin();
        //     i != configuration_map.end();
        //     i++)
        // {
        //     if((*i).second->active_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::FOLLOW_MASTER ||
        //        (*i).second->active_config.motor_config.invert_type == ck_ros_base_msgs_node::Motor_Config::OPPOSE_MASTER)
        //     {
        //         ck_ros_base_msgs_node::Motor motor;
        //         motor.controller_type = (*i).second->active_config.motor_config.controller_type;
        //         motor.id = (*i).second->active_config.motor_config.id;
        //         motor.control_mode = ck_ros_base_msgs_node::Motor::FOLLOWER;
        //         motor.output_value = (*i).second->active_config.master_id;
        //         motor_control_list.motors.push_back(motor);
        //     }
        // }

        // control_publisher.publish(motor_control_list);
    }

    static void motor_master_loop()
    {
        rclcpp::Rate timer(10);
        while(rclcpp::ok())
        {
            send_motor_configs();
            send_master_controls_periodic();
            // send_follower_controls();
            timer.sleep();
        }
    }

friend class Motor;
friend class MotorConfig;
};

std::map<uint8_t, MotorStatus *> MotorMaster::status_map;
std::map<uint8_t, MotorConfig *> MotorMaster::configuration_map;
std::map<uint8_t, Motor *> MotorMaster::motor_map;
std::thread * MotorMaster::motor_master_thread;
rclcpp::Publisher<ck_ros2_base_msgs_node::msg::MotorConfigurationArray>::SharedPtr MotorMaster::config_publisher;
rclcpp::Publisher<ck_ros2_base_msgs_node::msg::MotorControlArray>::SharedPtr MotorMaster::control_publisher;
rclcpp::Subscription<ck_ros2_base_msgs_node::msg::RobotStatus>::SharedPtr MotorMaster::robot_data_subscriber;
rclcpp::Subscription<ck_ros2_base_msgs_node::msg::MotorStatusArray>::SharedPtr MotorMaster::motor_status_subscriber;
std::atomic<int8_t> MotorMaster::robot_mode;

static MotorMaster * motor_master = nullptr;

void MotorConfig::apply()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->active_config = this->pending_config;
}

void MotorConfig::set_kP(double value, uint8_t slot)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if (this->pending_config.motor_config.k_p.size() > slot){
        this->pending_config.motor_config.k_p[slot] = value;
    }
}

void MotorConfig::set_kI(double value, uint8_t slot)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if (this->pending_config.motor_config.k_i.size() > slot){
        this->pending_config.motor_config.k_i[slot] = value;
    }
}

void MotorConfig::set_kD(double value, uint8_t slot)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if (this->pending_config.motor_config.k_d.size() > slot){
        this->pending_config.motor_config.k_d[slot] = value;
    }
}

void MotorConfig::set_kV(double value, uint8_t slot)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if (this->pending_config.motor_config.k_v.size() > slot){
        this->pending_config.motor_config.k_v[slot] = value;
    }
}

void MotorConfig::set_kS(double value, uint8_t slot)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if (this->pending_config.motor_config.k_s.size() > slot){
        this->pending_config.motor_config.k_s[slot] = value;
    }
}

void MotorConfig::set_motion_magic_cruise_velocity(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.motion_magic_cruise_velocity = value;
}

void MotorConfig::set_motion_magic_acceleration(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.motion_magic_acceleration = value;
}

void MotorConfig::set_motion_magic_jerk(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.motion_magic_jerk = value;
}

void MotorConfig::set_forward_soft_limit(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.forward_soft_limit_threshold = value;
}

void MotorConfig::set_forward_soft_limit_enable(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.enable_forward_soft_limit = enabled;
}

void MotorConfig::set_reverse_soft_limit(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.reverse_soft_limit_threshold = value;
}

void MotorConfig::set_reverse_soft_limit_enable(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.enable_reverse_soft_limit = enabled;
}

void MotorConfig::set_inverted(bool enabled)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.invert = enabled;

}

void MotorConfig::set_neutral_mode(MotorConfig::NeutralMode mode)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.brake_neutral = mode == MotorConfig::NeutralMode::BRAKE;
}

void MotorConfig::set_duty_cycle_open_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.duty_cycle_open_loop_ramp_period = value;
}

void MotorConfig::set_torque_current_open_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.torque_current_open_loop_ramp_period = value;
}

void MotorConfig::set_voltage_open_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.voltage_open_loop_ramp_period = value;
}

void MotorConfig::set_duty_cycle_closed_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.duty_cycle_closed_loop_ramp_period = value;
}

void MotorConfig::set_torque_current_closed_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.torque_current_closed_loop_ramp_period = value;
}

void MotorConfig::set_voltage_closed_loop_ramp(double value)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.voltage_closed_loop_ramp_period = value;
}

void MotorConfig::set_supply_current_limit(bool enabled, double current_limit, double trigger_current, double trigger_time)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.enable_supply_current_limit = enabled;
    this->pending_config.motor_config.supply_current_limit = current_limit;
    this->pending_config.motor_config.supply_current_threshold = trigger_current;
    this->pending_config.motor_config.supply_time_threshold = trigger_time;
}

void MotorConfig::set_stator_current_limit(bool enabled, double current_limit)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.enable_stator_current_limit = enabled;
    this->pending_config.motor_config.stator_current_limit = current_limit;
}

void MotorConfig::follow(uint8_t master_id)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    this->pending_config.motor_config.master_id = master_id;
}

void MotorConfig::set_defaults()
{
    this->follow(0);
    this->set_kP(0.0, 0);
    this->set_kI(0.0, 0);
    this->set_kD(0.0, 0);
    this->set_kV(0.0, 0);
    this->set_kS(0.0, 0);
    this->set_kP(0.0, 1);
    this->set_kI(0.0, 1);
    this->set_kD(0.0, 1);
    this->set_kV(0.0, 1);
    this->set_kS(0.0, 1);
    this->set_kP(0.0, 2);
    this->set_kI(0.0, 2);
    this->set_kD(0.0, 2);
    this->set_kV(0.0, 2);
    this->set_kS(0.0, 2);
    this->set_motion_magic_cruise_velocity(0.0);
    this->set_motion_magic_acceleration(0.0);
    this->set_motion_magic_jerk(0.0);
    this->set_forward_soft_limit(0.0);
    this->set_forward_soft_limit_enable(false);
    this->set_reverse_soft_limit(0.0);
    this->set_reverse_soft_limit_enable(false);
    this->set_inverted(false);
    this->set_neutral_mode(MotorConfig::NeutralMode::COAST);
    this->set_duty_cycle_open_loop_ramp(0.0);
    this->set_torque_current_open_loop_ramp(0.0);
    this->set_voltage_open_loop_ramp(0.0);
    this->set_duty_cycle_closed_loop_ramp(0.0);
    this->set_torque_current_closed_loop_ramp(0.0);
    this->set_voltage_closed_loop_ramp(0.0);
    this->set_supply_current_limit(true, 40.0, 0.0, 0.0);
    this->set_stator_current_limit(false, 0.0);
}

void MotorConfig::load_parameters(MotorConfigurationParameters params)
{
    this->pending_config.motor_config.master_id = params.master_id;
    this->pending_config.motor_config.k_p = params.k_p;
    this->pending_config.motor_config.k_i = params.k_i;
    this->pending_config.motor_config.k_d = params.k_d;
    this->pending_config.motor_config.k_v = params.k_v;
    this->pending_config.motor_config.k_s = params.k_s;
    this->pending_config.motor_config.motion_magic_cruise_velocity = params.motion_magic_cruise_velocity;
    this->pending_config.motor_config.motion_magic_acceleration = params.motion_magic_acceleration;
    this->pending_config.motor_config.motion_magic_jerk = params.motion_magic_jerk;
    this->pending_config.motor_config.forward_soft_limit_threshold = params.forward_soft_limit_threshold;
    this->pending_config.motor_config.enable_forward_soft_limit = params.enable_forward_soft_limit;
    this->pending_config.motor_config.reverse_soft_limit_threshold = params.reverse_soft_limit_threshold;
    this->pending_config.motor_config.enable_reverse_soft_limit = params.enable_reverse_soft_limit;
    this->pending_config.motor_config.invert = params.invert;
    this->pending_config.motor_config.brake_neutral = params.brake_neutral;
    this->pending_config.motor_config.duty_cycle_closed_loop_ramp_period = params.duty_cycle_closed_loop_ramp_period;
    this->pending_config.motor_config.torque_current_closed_loop_ramp_period = params.torque_current_closed_loop_ramp_period;
    this->pending_config.motor_config.voltage_closed_loop_ramp_period = params.voltage_closed_loop_ramp_period;
    this->pending_config.motor_config.duty_cycle_open_loop_ramp_period = params.duty_cycle_open_loop_ramp_period;
    this->pending_config.motor_config.torque_current_open_loop_ramp_period = params.torque_current_open_loop_ramp_period;
    this->pending_config.motor_config.voltage_open_loop_ramp_period = params.voltage_open_loop_ramp_period;
    this->pending_config.motor_config.enable_supply_current_limit = params.enable_supply_current_limit;
    this->pending_config.motor_config.supply_current_limit = params.supply_current_limit;
    this->pending_config.motor_config.supply_current_threshold = params.supply_current_threshold;
    this->pending_config.motor_config.supply_time_threshold = params.supply_time_threshold;
    this->pending_config.motor_config.enable_stator_current_limit = params.enable_stator_current_limit;
    this->pending_config.motor_config.stator_current_limit = params.stator_current_limit;
}

Motor::Motor(uint8_t id)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if(motor_master == nullptr)
    {
        motor_master = new MotorMaster();
    }
    this->id = id;
    motor_master->create_motor_config(id);
    motor_master->store_motor_pointer(id, this);
}

Motor::Motor(std::string id)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    if(motor_master == nullptr)
    {
        motor_master = new MotorMaster();
    }

    MotorConfigurationParameters params;

    try 
    {
       params.motor_id = node_handle->get_parameter(id+"_motor_id").as_int();
       params.master_id = node_handle->get_parameter(id+"_master_id").as_int();
       params.invert = node_handle->get_parameter(id+"_invert").as_bool();
       params.brake_neutral = node_handle->get_parameter(id+"_brake_neutral").as_bool();
       params.k_p = node_handle->get_parameter(id+"_k_p").as_double_array();
       params.k_i = node_handle->get_parameter(id+"_k_i").as_double_array();
       params.k_d = node_handle->get_parameter(id+"_k_d").as_double_array();
       params.k_v = node_handle->get_parameter(id+"_k_v").as_double_array();
       params.k_s = node_handle->get_parameter(id+"_k_s").as_double_array();
       params.enable_stator_current_limit = node_handle->get_parameter(id+"_enable_stator_current_limit").as_bool();
       params.stator_current_limit = node_handle->get_parameter(id+"_stator_current_limit").as_double();
       params.enable_supply_current_limit = node_handle->get_parameter(id+"_enable_supply_current_limit").as_bool();
       params.supply_current_limit = node_handle->get_parameter(id+"_supply_current_limit").as_double();
       params.supply_current_threshold = node_handle->get_parameter(id+"_supply_current_threshold").as_double();
       params.supply_time_threshold = node_handle->get_parameter(id+"_supply_time_threshold").as_double();
       params.duty_cycle_closed_loop_ramp_period = node_handle->get_parameter(id+"_duty_cycle_closed_loop_ramp_period").as_double();
       params.torque_current_closed_loop_ramp_period = node_handle->get_parameter(id+"_torque_current_closed_loop_ramp_period").as_double();
       params.voltage_closed_loop_ramp_period = node_handle->get_parameter(id+"_voltage_closed_loop_ramp_period").as_double();
       params.duty_cycle_open_loop_ramp_period = node_handle->get_parameter(id+"_duty_cycle_open_loop_ramp_period").as_double();
       params.torque_current_open_loop_ramp_period = node_handle->get_parameter(id+"_torque_current_open_loop_ramp_period").as_double();
       params.voltage_open_loop_ramp_period = node_handle->get_parameter(id+"_voltage_open_loop_ramp_period").as_double();
       params.enable_forward_soft_limit = node_handle->get_parameter(id+"_enable_forward_soft_limit").as_bool();
       params.forward_soft_limit_threshold = node_handle->get_parameter(id+"_forward_soft_limit_threshold").as_double();
       params.enable_reverse_soft_limit = node_handle->get_parameter(id+"_enable_reverse_soft_limit").as_bool();
       params.reverse_soft_limit_threshold = node_handle->get_parameter(id+"_reverse_soft_limit_threshold").as_double();
       params.motion_magic_cruise_velocity = node_handle->get_parameter(id+"_motion_magic_cruise_velocity").as_double();
       params.motion_magic_acceleration = node_handle->get_parameter(id+"_motion_magic_acceleration").as_double();
       params.motion_magic_jerk = node_handle->get_parameter(id+"_motion_magic_jerk").as_double();
    }
    catch (std::exception& e)
    {
        std::cout << "Error parsing motor parameters!" << std::endl;
        std::cout << e.what() << std::endl;
        throw e;
    }

    this->id = params.motor_id;
    motor_master->create_motor_config(params.motor_id);
    motor_master->store_motor_pointer(params.motor_id, this);
}

void Motor::set(ControlMode mode, double setpoint, FeedForwardType feed_forward_type, double feed_forward, uint8_t gain_slot)
{
    std::lock_guard<std::recursive_mutex> lock(mValueLock);
    mControlMode = mode;
    mOutput = setpoint;
    mArbFF = feed_forward;

    ck_ros2_base_msgs_node::msg::MotorControlArray motors;
    ck_ros2_base_msgs_node::msg::MotorControl motor;
    motor.id = id;
    motor.control_mode = (uint8_t) mode;
    motor.setpoint = setpoint;
    motor.feed_forward = feed_forward;
    motor.feed_forward_type = (uint8_t) feed_forward_type;
    motor.gain_slot = gain_slot;
    motors.motors.push_back(motor);
    static rclcpp::Publisher<ck_ros2_base_msgs_node::msg::MotorControlArray>::SharedPtr motor_control_pub = node_handle->create_publisher<ck_ros2_base_msgs_node::msg::MotorControlArray>("MotorControl", 50);
    motor_control_pub->publish(motors);
}

MotorConfig& Motor::config()
{
    return *(motor_master->retrieve_configuration(this->id));
}

MotorStatus& Motor::status()
{
    return *(motor_master->retrieve_status(this->id));
}

uint8_t MotorStatus::get_id()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.id;
}

double MotorStatus::get_sensor_position()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.sensor_position;
}

double MotorStatus::get_sensor_velocity()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.sensor_velocity;
}

double MotorStatus::get_bus_voltage()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.bus_voltage;
}

double MotorStatus::get_bus_current()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.bus_current;
}

double MotorStatus::get_stator_current()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.stator_current;
}

bool MotorStatus::get_forward_limit_closed()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.forward_limit_closed;
}

bool MotorStatus::get_reverse_limit_closed()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.reverse_limit_closed;
}

uint8_t MotorStatus::get_control_mode()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.control_mode;
}

double MotorStatus::get_commanded_output()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.commanded_output;
}

double MotorStatus::get_raw_output_percent()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    return status_data.raw_output_percent;
}

double MotorStatus::get_setpoint()
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    Motor* motor = MotorMaster::retrieve_motor(motor_id);
    return motor->mOutput;
}

bool MotorStatus::is_at_setpoint(double setpoint_delta_threshold)
{
    std::lock_guard<std::recursive_mutex> lock(motor_mutex);
    Motor* motor = MotorMaster::retrieve_motor(motor_id);

    double setpoint = motor->mOutput;

    switch (motor->mControlMode)
    {
        case Motor::ControlMode::MOTION_MAGIC:
        case Motor::ControlMode::POSITION:
        {
            return ck::math::within(setpoint, status_data.sensor_position, setpoint_delta_threshold);
        }
        case Motor::ControlMode::VELOCITY:
        {
            return ck::math::within(setpoint, status_data.sensor_velocity, setpoint_delta_threshold);
        }
        case Motor::ControlMode::TORQUE_CURRENT:
        {
            return ck::math::within(setpoint, status_data.bus_current, setpoint_delta_threshold);
        }
        default:
        {
            return true;
        }
    }
}