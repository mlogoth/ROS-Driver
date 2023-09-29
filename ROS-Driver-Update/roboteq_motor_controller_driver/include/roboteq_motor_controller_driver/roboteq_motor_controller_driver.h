#pragma once

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
	void run();
	bool configservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Config::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Config::Response> response);
	bool commandservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Command::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Command::Response> response);
	bool maintenanceservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Maintenance::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Maintenance::Response> response);
	
	std::shared_ptr<ParamListener> param_listener_;
    Params params_;

	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd_vel_channel_1_sub;
	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd_vel_channel_2_sub;

	rclcpp::Service<roboteq_motor_controller_msgs::srv::Config>::SharedPtr config_srv;
	rclcpp::Service<roboteq_motor_controller_msgs::srv::Command>::SharedPtr command_srv;
	rclcpp::Service<roboteq_motor_controller_msgs::srv::Maintenance>::SharedPtr maintenance_srv;

	rclcpp::Publisher<std_msgs::msg::String>::SharedPtr serial_read_pub_;
	
	serial::Serial ser_;


	rclcpp::Time previous_time;
	
	
	
	// int max_rpm = 3500;
	// double reduction_ratio;
	// ros::Publisher read_publisher;
	

	
	// int rate;
	// ros::Time previous_time;

	// ros::NodeHandle nh;

	// // added in changes
	// ros::NodeHandle nh_priv_;
	// std::vector<int> f_list; // a list of frequencies for the queries to be published
	// std::vector<ros::Publisher> query_pub_;
	// ros::NodeHandle nh_;
	// ros::Timer timer_pub_;
	
	// std::mutex locker;
	// ros::Publisher serial_read_pub_;

	// std::vector<int> cum_query_size;

	// void queryCallback(const ros::TimerEvent &);

	// void formQuery(std::string,
	// 			   std::map<std::string, std::string> &,
	// 			   std::vector<ros::Publisher> &,
	// 			   std::stringstream &);

	
	
	// void rpm_mapping(const double &right_speed, const double &left_speed, double &right_speed_cr, double &left_speed_cr);
	// void skid_steering_vel_callback(const geometry_msgs::Twist &msg);
	// void dual_vel_callback(const std_msgs::Int16 &msg);
	// void channel_1_vel_callback(const std_msgs::Int16 &msg);
	// void channel_2_vel_callback(const std_msgs::Int16 &msg);
	
	
	
};

} // roboteq namespace