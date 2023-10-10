#pragma once

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <geometry_msgs/msg/twist.hpp>
#include <example_interfaces/msg/int16.hpp>
#include <example_interfaces/msg/string.hpp>
#include <example_interfaces/msg/int64_multi_array.hpp>
#include "roboteq_motor_controller_driver_parameters.hpp"
#include "roboteq_motor_controller_msgs/srv/config.hpp"
#include "roboteq_motor_controller_msgs/srv/command.hpp"
#include "roboteq_motor_controller_msgs/srv/maintenance.hpp"

template <typename T> float sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

namespace roboteq
{
class RoboteqDriver : public rclcpp::Node
{

public:
	RoboteqDriver();
	~RoboteqDriver();

private:
	void connect();
	void initialize_services();
	void rpm_mapping(const double &right_speed, const double &left_speed, double &right_speed_cr, double &left_speed_cr);
	void skid_steering_vel_callback(const geometry_msgs::msg::Twist &msg);
	void channel_1_vel_callback(const example_interfaces::msg::Int16 &msg);
	void channel_2_vel_callback(const example_interfaces::msg::Int16 &msg);
	void queryCallback();
	void formQuery(const int index, std::stringstream &ser_str);
	void run();
	bool configservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Config::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Config::Response> response);
	bool commandservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Command::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Command::Response> response);
	bool maintenanceservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Maintenance::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Maintenance::Response> response);
	
	std::shared_ptr<ParamListener> param_listener_;
    Params params_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
	rclcpp::Subscription<example_interfaces::msg::Int16>::SharedPtr cmd_vel_channel_1_sub;
	rclcpp::Subscription<example_interfaces::msg::Int16>::SharedPtr cmd_vel_channel_2_sub;

	rclcpp::Service<roboteq_motor_controller_msgs::srv::Config>::SharedPtr config_srv;
	rclcpp::Service<roboteq_motor_controller_msgs::srv::Command>::SharedPtr command_srv;
	rclcpp::Service<roboteq_motor_controller_msgs::srv::Maintenance>::SharedPtr maintenance_srv;

	rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr serial_read_pub_;
	std::vector<rclcpp::Publisher<example_interfaces::msg::Int64MultiArray>::SharedPtr> query_pubs_;

	rclcpp::TimerBase::SharedPtr queries_timer_;
	
	serial::Serial ser_;

	std::vector<int> cum_query_size{0};

	bool both_count_and_speed_{false};
	int both_count_and_speed_freq_index_;
	int count_index_;
	int speed_index_;	
};

} // roboteq namespace