#include <roboteq_motor_controller_driver/diff_odom.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("RoboteqOdom");

namespace diff_odom
{

OdometryCalc::OdometryCalc() : Node("diff_odom")
{

	// Get Parameters
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();

	// Print parameters
	RCLCPP_INFO_STREAM(LOGGER, "encoder_cpr: " << params_.encoder.encoder_cpr);
	RCLCPP_INFO_STREAM(LOGGER, "encoder_min: " << params_.encoder.encoder_min);
	RCLCPP_INFO_STREAM(LOGGER, "encoder_max: " << params_.encoder.encoder_max);
	RCLCPP_INFO_STREAM(LOGGER, "wrp_lim: " << params_.encoder.wrp_lim);
	RCLCPP_INFO_STREAM(LOGGER, "sub_to_abs: " << params_.encoder.sub_to_abs);		

	RCLCPP_INFO_STREAM(LOGGER, "reduction_ratio: " << params_.mechanical.reduction_ratio);
	RCLCPP_INFO_STREAM(LOGGER, "wheel_circumference: " << params_.mechanical.wheel_circumference);
	RCLCPP_INFO_STREAM(LOGGER, "track_width: " << params_.mechanical.track_width);
	
	RCLCPP_INFO_STREAM(LOGGER, "tf_publish: " << params_.tf.tf_publish);
	RCLCPP_INFO_STREAM(LOGGER, "tf_header_frame: " << params_.tf.tf_header_frame);
	RCLCPP_INFO_STREAM(LOGGER, "tf_child_frame: " << params_.tf.tf_child_frame);
	RCLCPP_INFO_STREAM(LOGGER, "odom_topic: " << params_.odom.odom_topic);
	RCLCPP_INFO_STREAM(LOGGER, "odom_frame: " << params_.odom.odom_frame);	

	init_variables();

	rclcpp::SubscriptionOptions options;
	encoder_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	options.callback_group = encoder_group_;

	wheel_sub = this->create_subscription<example_interfaces::msg::Int64MultiArray>(params_.ns + "/count_and_speed", rclcpp::SystemDefaultsQoS(), std::bind(&OdometryCalc::roboteq_callback, this, _1), options);

	if (params_.tf.tf_publish)
	{
		odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	}
	
	odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(params_.odom.odom_topic, rclcpp::SystemDefaultsQoS());

	RCLCPP_INFO(LOGGER, "Started odometry computing node");
}

void OdometryCalc::init_variables()
{
	abs_init = false;	

	encoder_low_wrap = ((params_.encoder.encoder_max - params_.encoder.encoder_min) * params_.encoder.wrp_lim) + params_.encoder.encoder_min;
	encoder_high_wrap = ((params_.encoder.encoder_max - params_.encoder.encoder_min) * (1.0 - params_.encoder.wrp_lim)) + params_.encoder.encoder_min;

	x_final = 0.0;
	y_final = 0.0;
	theta_final = 0.0;	

	//if we are subscribing to the absolute topic, we are receiving pulses instead of counts, which are 4 times more.
	//E.g: 24 pulses per motor revolution correspond to 6 counts p.m.r.
	//Mind to set the counts per revolution in the query.yaml file, not the pulses.
	meters_per_tick = params_.mechanical.wheel_circumference/(params_.encoder.encoder_cpr*params_.mechanical.reduction_ratio); 
	if (params_.encoder.sub_to_abs)
	{
		meters_per_tick = meters_per_tick / 4;
	}

	if (params_.tf.tf_publish)
	{
		odom_trans.header.frame_id = params_.tf.tf_header_frame; //originally was set to odom
		odom_trans.child_frame_id = params_.tf.tf_child_frame;
		odom_trans.transform.translation.z = 0.0;
	}	
}

//Encoder callback
void OdometryCalc::roboteq_callback(const example_interfaces::msg::Int64MultiArray &msg)
{
	left_count = msg.data[0];
	right_count = msg.data[1];
	left_rpm = msg.data[2];
	right_rpm = msg.data[3];	
	update();
	OdomPub();
	if (params_.tf.tf_publish)
	{
		TfPub();
	}
}

//Update function
void OdometryCalc::update()
{
	//left and right covered distances
	double d_left, d_right;
	// left and right speed
	double left_speed, right_speed;
	//Rover distance covered and angle change
	double DeltaS, DeltaTh;
	//Distance covered in X and Y axes
	double DeltaX, DeltaY;

	////////////////////////////////////////////////////////////////////////////////////
	//In case we read absolute values, we need to calculate the actual elapsed pulses //
	////////////////////////////////////////////////////////////////////////////////////
	if (params_.encoder.sub_to_abs)
	{
		if (!abs_init)
		{
			RCLCPP_INFO_STREAM(LOGGER, "Initializing absolute pulse values.");
			prev_l_pulses = left_count;
			prev_r_pulses = right_count;
			abs_init = true;
		}
		left_count_abs = left_count;
		right_count_abs = right_count;

		//check for value wrapping, in case we exceeded +-2147M

		//left encoder
		if ((left_count_abs < encoder_low_wrap) && (prev_l_pulses > encoder_high_wrap))
		{
			left_count = (params_.encoder.encoder_max - prev_l_pulses) + (left_count_abs - params_.encoder.encoder_min);
		}
		else if ((left_count_abs > encoder_high_wrap) && (prev_l_pulses < encoder_low_wrap))
		{
			left_count = (params_.encoder.encoder_max - left_count_abs) + (prev_l_pulses - params_.encoder.encoder_min);
		}
		else
		{
			left_count = left_count_abs - prev_l_pulses;
		}

		//right encoder
		if ((right_count_abs < encoder_low_wrap) && (prev_r_pulses > encoder_high_wrap))
		{
			right_count = (params_.encoder.encoder_max - prev_r_pulses) + (right_count_abs - params_.encoder.encoder_min);
		}
		else if ((right_count_abs > encoder_high_wrap) && (prev_r_pulses < encoder_low_wrap))
		{
			right_count = (params_.encoder.encoder_max - right_count_abs) + (prev_r_pulses - params_.encoder.encoder_min);
		}
		else
		{
			right_count = right_count_abs - prev_r_pulses;
		}
		
		//setting the old abs values
		prev_l_pulses = left_count_abs;
		prev_r_pulses = right_count_abs;
	}

	//////////////////////////////////////////////////
	//calculate the distances covered left and right//
	//////////////////////////////////////////////////
	d_left = left_count * meters_per_tick;
	d_right = right_count * meters_per_tick;

	//////////////////////////////////////////////////
	//calculate the left and right speed//
	//////////////////////////////////////////////////
	left_speed = (left_rpm * params_.mechanical.wheel_circumference) / (60.0 * params_.mechanical.reduction_ratio);
	right_speed = (right_rpm * params_.mechanical.wheel_circumference) / (60.0 * params_.mechanical.reduction_ratio);

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//calculate the distance and angle covered, as well as the rates of change and total displacements//
	////////////////////////////////////////////////////////////////////////////////////////////////////

	//linear
	DeltaS = (d_left + d_right) / 2.0;
	dx = (left_speed + right_speed) / 2.0;

	//angular
	DeltaTh = (d_right - d_left) / params_.mechanical.track_width;
	dr = (right_speed - left_speed) / params_.mechanical.track_width;
		
	if (DeltaS != 0)
	{
		DeltaX = DeltaS * cos(theta_final + (DeltaTh / 2));
		DeltaY = DeltaS * sin(theta_final + (DeltaTh / 2));
		x_final = x_final + DeltaX;
		y_final = y_final + DeltaY;
	}

	theta_final = theta_final + DeltaTh;
	//wrap angle values between -180 and 180 degrees
	if (theta_final > M_PI) 
	{
		theta_final = theta_final - 2*M_PI;
	}
	else if (theta_final < - M_PI) 
	{
		theta_final = theta_final + 2*M_PI;
	}
}

void OdometryCalc::OdomPub()
{
	///////////////////////////////
	//set up and publish odometry//
	///////////////////////////////
	tf2::Quaternion q;
	q.setRPY(0,0,theta_final);
	odom_quat = tf2::toMsg(q);
	nav_msgs::msg::Odometry odom;
	odom.header.stamp = this->get_clock()->now();
	odom.header.frame_id = params_.odom.odom_frame;
	odom.pose.pose.position.x = x_final;
	odom.pose.pose.position.y = y_final;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	odom.child_frame_id = params_.tf.tf_child_frame;
	odom.twist.twist.linear.x = dx;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.angular.z = dr;
	odom_pub->publish(odom);
}

void OdometryCalc::TfPub()
{
	/////////////////////////////////////
	//set up and publish transformation//
	/////////////////////////////////////
	odom_trans.header.stamp = this->get_clock()->now();
	odom_trans.transform.translation.x = x_final;
	odom_trans.transform.translation.y = y_final;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster->sendTransform(odom_trans);
}

} // namespace diff_odom

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::executors::SingleThreadedExecutor executor;
	diff_odom::OdometryCalc odom_node{};
	executor.add_node(odom_node.get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();
  	return 0;
}

