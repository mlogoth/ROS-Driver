#include <roboteq_motor_controller_driver/diff_odom.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using std::placeholders::_1;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("RoboteqOdom");

OdometryCalc::OdometryCalc() : Node("diff_odom")
{
	// Declare parameters and their default values
	/////////////////////////////
	//Encoder related variables//
	/////////////////////////////
	this->declare_parameter("encoder_cpr", 24);
	this->declare_parameter("encoder_min", -2147483648.0);
	this->declare_parameter("encoder_max", 2147483648.0);
	this->declare_parameter("wrp_lim", 0.3);
	this->declare_parameter("sub_to_abs", false);

	//////////////////////
	//General parameters//
	//////////////////////
	this->declare_parameter("reduction_ratio", 70);
	this->declare_parameter("wheel_circumference", 1.1623895);
	this->declare_parameter("track_width", 0.9925);
	this->declare_parameter("enable_imu_yaw", false);
	this->declare_parameter("imu_topic", "imu/data");
	this->declare_parameter("imu_discarded", 100);
	this->declare_parameter("tf_publish", true);
	this->declare_parameter("tf_header_frame", "map");
	this->declare_parameter("tf_child_frame", "base_footprint");
	this->declare_parameter("odom_topic", "odom");	
	this->declare_parameter("odom_frame", "odom");

	// Get parameters
	encoder_cpr = this->get_parameter("encoder_cpr").as_int();
	encoder_min = this->get_parameter("encoder_min").as_double();
	encoder_max = this->get_parameter("encoder_max").as_double();
	wrp_lim = this->get_parameter("wrp_lim").as_double();
	sub_to_abs = this->get_parameter("sub_to_abs").as_bool();

	reduction_ratio = this->get_parameter("reduction_ratio").as_int();
	wheel_circumference = this->get_parameter("wheel_circumference").as_double();
	track_width = this->get_parameter("track_width").as_double();
	enable_imu_yaw = this->get_parameter("enable_imu_yaw").as_bool();
	imu_topic = this->get_parameter("imu_topic").as_string();
	imu_discarded = this->get_parameter("imu_discarded").as_int();
	tf_publish = this->get_parameter("tf_publish").as_bool();
	tf_header_frame = this->get_parameter("tf_header_frame").as_string();
	tf_child_frame = this->get_parameter("tf_child_frame").as_string();
	odom_topic = this->get_parameter("odom_topic").as_string();	
	odom_frame = this->get_parameter("odom_frame").as_string();

	// Print parameters
	RCLCPP_INFO_STREAM(LOGGER, "encoder_cpr: " << encoder_cpr);
	RCLCPP_INFO_STREAM(LOGGER, "encoder_min: " << encoder_min);
	RCLCPP_INFO_STREAM(LOGGER, "encoder_max: " << encoder_max);
	RCLCPP_INFO_STREAM(LOGGER, "wrp_lim: " << wrp_lim);
	RCLCPP_INFO_STREAM(LOGGER, "sub_to_abs: " << sub_to_abs);		

	RCLCPP_INFO_STREAM(LOGGER, "reduction_ratio: " << reduction_ratio);
	RCLCPP_INFO_STREAM(LOGGER, "wheel_circumference: " << wheel_circumference);
	RCLCPP_INFO_STREAM(LOGGER, "track_width: " << track_width);
	RCLCPP_INFO_STREAM(LOGGER, "enable_imu_yaw: " << enable_imu_yaw);
	RCLCPP_INFO_STREAM(LOGGER, "imu_topic: " << imu_topic);
	RCLCPP_INFO_STREAM(LOGGER, "imu_discarded: " << imu_discarded);
	RCLCPP_INFO_STREAM(LOGGER, "tf_publish: " << tf_publish);
	RCLCPP_INFO_STREAM(LOGGER, "tf_header_frame: " << tf_header_frame);
	RCLCPP_INFO_STREAM(LOGGER, "tf_child_frame: " << tf_child_frame);
	RCLCPP_INFO_STREAM(LOGGER, "odom_topic: " << odom_topic);
	RCLCPP_INFO_STREAM(LOGGER, "odom_frame: " << odom_frame);	

	init_variables();

	rclcpp::SubscriptionOptions options;
	encoder_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
	options.callback_group = encoder_group_;

	const std::string encoder_topic = sub_to_abs ? "abs_hall_count" : "hall_count";
	wheel_sub = this->create_subscription<roboteq_motor_controller_msgs::msg::ChannelValues>(encoder_topic, rclcpp::SystemDefaultsQoS(), std::bind(&OdometryCalc::encoderBCR, this, _1), options);
		
	if (enable_imu_yaw)
	{
		init_imu_variables();
		imu_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
		options.callback_group = imu_group_;
		imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(imu_topic, rclcpp::SystemDefaultsQoS(), std::bind(&OdometryCalc::imu_callback, this, _1), options);
	}

	if (tf_publish)
	{
		odom_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	}
	
	odom_pub = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::SystemDefaultsQoS());

	RCLCPP_INFO(LOGGER, "Started odometry computing node");
}

void OdometryCalc::init_variables()
{
	abs_init = false;	

	encoder_low_wrap = ((encoder_max - encoder_min) * wrp_lim) + encoder_min;
	encoder_high_wrap = ((encoder_max - encoder_min) * (1.0 - wrp_lim)) + encoder_min;
	then = this->get_clock()->now();

	x_final = 0.0;
	y_final = 0.0;
	theta_final = 0.0;	

	//if we are subscribing to the absolute topic, we are receiving pulses instead of counts, which are 4 times more.
	//E.g: 24 pulses per motor revolution correspond to 6 counts p.m.r.
	//Mind to set the counts per revolution in the query.yaml file, not the pulses.
	meters_per_tick = wheel_circumference/(encoder_cpr*reduction_ratio); 
	if (sub_to_abs)
	{
		meters_per_tick = meters_per_tick / 4;
	}

	if (tf_publish)
	{
		odom_trans.header.frame_id = tf_header_frame; //originally was set to odom
		odom_trans.child_frame_id = tf_child_frame;
		odom_trans.transform.translation.z = 0.0;
	}	
}

void OdometryCalc::init_imu_variables()
{
	imu_initialized = false;
	imu_yaw_init = 0.0;
	imu_yaw = 0.0;

}

//Encoder callback
void OdometryCalc::encoderBCR(const roboteq_motor_controller_msgs::msg::ChannelValues &msg)
{
	right_count = msg.value[0];
	left_count = msg.value[1];
	update();
	OdomPub();
	if (tf_publish)
	{
		TfPub();
	}
}

//Imu callback
void OdometryCalc::imu_callback(const sensor_msgs::msg::Imu &msg)
{
	prev_imu_yaw = imu_yaw;
	//TF quaternion
	tf2::Quaternion q{msg.orientation.x,
					  msg.orientation.y,
					  msg.orientation.z,
					  msg.orientation.w};

	// TF matrix
	tf2::Matrix3x3 m{q};

	// Calculate ROLL PITCH YAW from quaternion
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	
	// Check if yaw is NaN: When yaw= NaN, then the statement yaw!=yaw is always True
	if (yaw == yaw)
	{
		if (!imu_initialized)
		{
			imu_yaw_init = imu_yaw_init + yaw;
			imu_cnt++;
			if (imu_cnt >= imu_discarded)
			{
				imu_initialized = true;
				imu_yaw_init /= static_cast<double>(imu_cnt);
			}
		}
		else
		{
			{
				const std::lock_guard<std::mutex> yaw_lock(yaw_mtx);
				imu_yaw = yaw - imu_yaw_init;
			}
		}
	}
}

//Update function
void OdometryCalc::update()
{
	////////////////////////////////////////////////////////////////////////////////////
	//In case we read absolute values, we need to calculate the actual elapsed pulses //
	////////////////////////////////////////////////////////////////////////////////////
	if (sub_to_abs)
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
			left_count = (encoder_max - prev_l_pulses) + (left_count_abs - encoder_min);
		}
		else if ((left_count_abs > encoder_high_wrap) && (prev_l_pulses < encoder_low_wrap))
		{
			left_count = (encoder_max - left_count_abs) + (prev_l_pulses - encoder_min);
		}
		else
		{
			left_count = left_count_abs - prev_l_pulses;
		}

		//right encoder
		if ((right_count_abs < encoder_low_wrap) && (prev_r_pulses > encoder_high_wrap))
		{
			right_count = (encoder_max - prev_r_pulses) + (right_count_abs - encoder_min);
		}
		else if ((right_count_abs > encoder_high_wrap) && (prev_r_pulses < encoder_low_wrap))
		{
			right_count = (encoder_max - right_count_abs) + (prev_r_pulses - encoder_min);
		}
		else
		{
			right_count = right_count_abs - prev_r_pulses;
		}
		
		//setting the old abs values
		prev_l_pulses = left_count_abs;
		prev_r_pulses = right_count_abs;
	}

	//////////////////////////////////////
	//calculate current and elapsed time//
	//////////////////////////////////////
	now = this->get_clock()->now();
	elapsed = now.seconds() - then.seconds();

	//////////////////////////////////////////////////
	//calculate the distances covered left and right//
	//////////////////////////////////////////////////
	d_left = left_count * meters_per_tick;
	d_right = right_count * meters_per_tick;

	////////////////////////////////////////////////////////////////////////////////////////////////////
	//calculate the distance and angle covered, as well as the rates of change and total displacements//
	////////////////////////////////////////////////////////////////////////////////////////////////////

	//linear
	DeltaS = (d_left + d_right) / 2.0;
	dx = DeltaS / elapsed;
	if (DeltaS != 0)
	{
		DeltaX = DeltaS * cos(theta_final + (DeltaTh / 2));
		DeltaY = DeltaS * sin(theta_final + (DeltaTh / 2));
		x_final = x_final + DeltaX;
		y_final = y_final + DeltaY;
	}

	//angular
	if ((!enable_imu_yaw) || (enable_imu_yaw && !imu_initialized))
	{
		DeltaTh = (d_left - d_right) / track_width; // th is tan(th)
		dr = DeltaTh / elapsed;
		theta_final = theta_final + DeltaTh;
	}
	else
	{
		{
			const std::lock_guard<std::mutex> yaw_lock(yaw_mtx);
			theta_final = imu_yaw;
    		dr = (imu_yaw - prev_imu_yaw)/elapsed;
		}		
	}

	//wrap angle values between -180 and 180 degrees
	if (theta_final > M_PI) 
	{
		theta_final = theta_final - 2*M_PI;
	}
	else if (theta_final < - M_PI) 
	{
		theta_final = theta_final + 2*M_PI;
	}

	////////////////////////
	//update previous time//
	////////////////////////
	then = now;

	////////////////
	//debug prints//
	////////////////
	// RCLCPP_INFO_STREAM("x final: " << x_final);
	// RCLCPP_INFO_STREAM("y final: " << y_final);
	// RCLCPP_INFO_STREAM("theta_final: " << theta_final*180.0/3.14);
	// RCLCPP_INFO_STREAM("left_abs_hall_count: " << left_count_abs);
	// RCLCPP_INFO_STREAM("right_abs_hall_count: " << right_count_abs);
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
	odom.header.stamp = now;
	odom.header.frame_id = odom_frame;
	odom.pose.pose.position.x = x_final;
	odom.pose.pose.position.y = y_final;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = odom_quat;
	odom.child_frame_id = tf_child_frame;
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
	odom_trans.header.stamp = now;
	odom_trans.transform.translation.x = x_final;
	odom_trans.transform.translation.y = y_final;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster->sendTransform(odom_trans);
}

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	executor.add_node(std::make_shared<OdometryCalc>());
	executor.spin();
	rclcpp::shutdown();
  	return 0;
}