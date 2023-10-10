#pragma once

#include <rclcpp/rclcpp.hpp>
#include "diff_odom_parameters.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <example_interfaces/msg/int64_multi_array.hpp>
#include "tf2_ros/transform_broadcaster.h"

namespace diff_odom
{
class OdometryCalc : public rclcpp::Node
{

public:
	OdometryCalc();

private:
	void init_variables();	
	void roboteq_callback(const example_interfaces::msg::Int64MultiArray &msg);
	void update();
	void TfPub();
	void OdomPub();	

	std::shared_ptr<diff_odom::ParamListener> param_listener_;
    diff_odom::Params params_;

	rclcpp::CallbackGroup::SharedPtr encoder_group_;
	rclcpp::CallbackGroup::SharedPtr imu_group_;

	rclcpp::Subscription<example_interfaces::msg::Int64MultiArray>::SharedPtr wheel_sub;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

	std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
	geometry_msgs::msg::TransformStamped odom_trans;
	geometry_msgs::msg::Quaternion odom_quat;
	
	//count or pulse values received from the subscriber, depending on where we are subscribing
	double left_count;
	double right_count;
	double left_rpm;
	double right_rpm;
	//previous pulse values, used to calculate the elapsed pulses in case we are subscribing to the absolute topic
	int prev_l_pulses;
	int prev_r_pulses;
	//Absolute count values, used only in case we are subscribing to the absolute topic
	int left_count_abs;
	int right_count_abs;
	//Variables for wrapping
	double encoder_low_wrap; 
	double encoder_high_wrap;
	//boolean for the initialization of absolute pulse values in case of absolute subscription
	bool abs_init;
	//meters covered per each encoder tick
	double meters_per_tick;
	//final x,y,th
	double x_final, y_final, theta_final;
	//distance and angle rate of change
	double dx, dr; 
};

} // namespace diff_odom

