#pragma once

#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include "diff_odom_parameters.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "roboteq_motor_controller_msgs/msg/channel_values.hpp"

namespace diff_odom
{
class OdometryCalc : public rclcpp::Node
{

public:
	OdometryCalc();

private:
	void init_variables();
	void init_imu_variables();	
	void encoderBCR(const roboteq_motor_controller_msgs::msg::ChannelValues &msg);
	void imu_callback(const sensor_msgs::msg::Imu &msg);	
	void update();
	void TfPub();
	void OdomPub();	

	std::shared_ptr<diff_odom::ParamListener> param_listener_;
    diff_odom::Params params_;

	rclcpp::CallbackGroup::SharedPtr encoder_group_;
	rclcpp::CallbackGroup::SharedPtr imu_group_;

	rclcpp::Subscription<roboteq_motor_controller_msgs::msg::ChannelValues>::SharedPtr wheel_sub;
	rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;

	std::unique_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
	geometry_msgs::msg::TransformStamped odom_trans;
	geometry_msgs::msg::Quaternion odom_quat;

	std::mutex yaw_mtx;
	
	//count or pulse values received from the subscriber, depending on where we are subscribing
	double left_count;
	double right_count;
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
	bool abs_init ; 

	//Time variables
	rclcpp::Time then;
	rclcpp::Time now;
	double elapsed;

	//Distance related variables//
	//left and right covered distances
	double d_left, d_right; 
	//Rover distance covered and angle change
	double DeltaS, DeltaTh;
	//Distance covered in X and Y axes
	double DeltaX, DeltaY;
	//distance and angle rate of change
	double dx, dr;
	//meters covered per each encoder tick
	double meters_per_tick;
	//final x,y,th
	double x_final, y_final, theta_final;

	//imu variables
	bool imu_initialized;
	double prev_imu_yaw, imu_yaw, imu_yaw_init;
	int imu_cnt;	
};

} // namespace diff_odom

