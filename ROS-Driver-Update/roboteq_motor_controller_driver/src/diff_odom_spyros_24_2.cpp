#include "ros/ros.h"
#include "ros/console.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <math.h>
#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>
#include <string>
#include <sstream>

class Odometry_calc
{

public:
	Odometry_calc(ros::NodeHandle handle);
	void spin();
	void imu_setup();
	void imu_callback(const sensor_msgs::Imu &imu);


private:
	ros::NodeHandle n;
	ros::NodeHandle pn;
	ros::Subscriber wheel_sub;
	ros::Subscriber imu_sub;
	ros::Publisher odom_pub;
	tf::TransformBroadcaster odom_broadcaster;
	geometry_msgs::Quaternion odom_quat;

	/////////////////////
	//Exposed variables//
	/////////////////////
	int encoder_cpr;
	int encoder_min;
	int encoder_max;
	int reduction_ratio;
	double wheel_circumference;
	double track_width;
	int rate;
	bool enable_imu_yaw;
	std::string imu_topic;
	bool wrapping_enabled;
	double wrp_lim; 
	bool sub_to_abs;	// subscribe to abs_hall_count instead of hall_count topic to receive the total-absolute number of pulses instead of the relative number of counts
	std::string odom_topic;
	std::string tf_header_frame;
	std::string tf_child_frame;
	bool tf_publish;
	int imu_discarded;
	std::string odom_frame ;
	
	//count or pulse values received from the subscriber, depending on where we are subscribing
	int left_count;
	int right_count;
	//previous pulse values, used to calculate the elapsed pulses in case we are subscribing to the absolute topic
	int prev_l_pulses;
	int prev_r_pulses;
	//
	//Absolute count values, calculated only in case we are subscribing to the relative topic
	int left_count_abs;
	int right_count_abs;
	//Variables for wrapping
	int prev_l_enc;
	int prev_r_enc;
	double encoder_low_wrap; 
	double encoder_high_wrap;
	int lmult;
	int rmult;
	//boolean for the initialization of absolute pulse values in case of absolute subscription
	bool abs_init ; 

	//Time variables
	ros::Time then;
	ros::Time now;
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

	//funcs and callbacks
	void encoderBCR(const roboteq_motor_controller_driver::channel_values &left_ticks);
	void init_variables();
	void update();
	void TfPub();
	void OdomPub();
};

Odometry_calc::Odometry_calc(ros::NodeHandle pn)
{
	/////////////////////////////
	//Encoder related variables//
	/////////////////////////////
	pn.param("encoder_cpr", encoder_cpr, 24);
	ROS_INFO_STREAM("encoder_cpr: " << encoder_cpr);	
	pn.param("encoder_min", encoder_min, -65536);
	ROS_INFO_STREAM("encoder_min: " << encoder_min);
	pn.param("encoder_max", encoder_max, 65536);
	ROS_INFO_STREAM("encoder_max: " << encoder_max);
	pn.param("wrapping_enabled", wrapping_enabled, false);
 	ROS_INFO_STREAM("wrapping_enabled: " << wrapping_enabled);
	pn.param("wrp_lim", wrp_lim, 0.3);
 	ROS_INFO_STREAM("wrp_lim: " << wrp_lim);
	pn.param("sub_to_abs", sub_to_abs, false);
 	ROS_INFO_STREAM("sub_to_abs: " << sub_to_abs);

	//////////////////////
	//General parameters//
	//////////////////////
	pn.param("reduction_ratio", reduction_ratio, 70);
	ROS_INFO_STREAM("reduction_ratio: " << reduction_ratio);
	pn.param("wheel_circumference", wheel_circumference, 1.1623895);
	ROS_INFO_STREAM("wheel_circumference: " << wheel_circumference);
	pn.param("track_width", track_width, 0.9925);
 	ROS_INFO_STREAM("track_width: " << track_width);
	pn.param("enable_imu_yaw", enable_imu_yaw, false);
 	ROS_INFO_STREAM("enable_imu_yaw: " << enable_imu_yaw);
 	pn.param<std::string>("imu_topic", imu_topic, "imu/data");
	ROS_INFO_STREAM("imu_topic: " << imu_topic);

	pn.param("imu_discarded", imu_discarded, 100);
	ROS_INFO_STREAM("imu_discarded: " << imu_discarded);
	pn.param("tf_publish", tf_publish, true);
	ROS_INFO_STREAM("tf_publish: " << tf_publish);
	pn.param<std::string>("odom_topic", odom_topic, "odom");
	ROS_INFO_STREAM("odom_topic: " << odom_topic);
 	pn.param<std::string>("tf_child_frame", tf_child_frame, "base_footprint");
	ROS_INFO_STREAM("tf_child_frame: " << tf_child_frame);
 	pn.param<std::string>("tf_header_frame", tf_header_frame, "map");
	ROS_INFO_STREAM("tf_header_frame: " << tf_header_frame);

	pn.param<std::string>("odom_frame", odom_frame, "odom");
	ROS_INFO_STREAM("odom_frame: " << odom_frame);

	//////////////////////////
	//Time related variables//
	//////////////////////////
	pn.param("rate", rate, 50);
	ROS_INFO_STREAM("rate: " << rate);

	init_variables();
	ROS_INFO("Started odometry computing node");
	if (sub_to_abs)
	{
		wheel_sub = pn.subscribe("abs_hall_count", 1, &Odometry_calc::encoderBCR, this);
	}
	else
	{
		wheel_sub = pn.subscribe("hall_count", 1, &Odometry_calc::encoderBCR, this);
	}
	odom_pub = n.advertise<nav_msgs::Odometry>(odom_topic, 50);
	if (enable_imu_yaw)
	{
		imu_setup();
	}
}

void Odometry_calc::init_variables()
{
	left_count_abs = 0;
	right_count_abs = 0;
	abs_init = false;
	
	if (wrapping_enabled)
	{
		prev_l_enc = 0;
		prev_r_enc = 0;
		lmult = 0;
		rmult = 0;
		encoder_low_wrap = ((encoder_max - encoder_min) * wrp_lim) + encoder_min;
		encoder_high_wrap = ((encoder_max - encoder_min) * (1.0 - wrp_lim)) + encoder_min;
	}

	then = ros::Time::now();

	x_final = 0.0;
	y_final = 0.0;
	theta_final = 0.0;
	meters_per_tick = wheel_circumference/(encoder_cpr*reduction_ratio);
	//if we are subscribing to the absolute topic, we are receiving pulses instead of counts, which are 4 times more.
	//E.g: 24 pulses per motor revolution correspond to 6 counts p.m.r.
	//Mind to set the counts per revolution in the query.yaml file, not the pulses. 
	if (sub_to_abs)
	{
		meters_per_tick = meters_per_tick / 4;
	}
}

//Update function
void Odometry_calc::update()
{
	/////////////////////////////////////////////////////////////////////////
	//In case we read absolute values, we need to calculate the pulses read//
	/////////////////////////////////////////////////////////////////////////
	if (sub_to_abs)
	{
		if (!abs_init)
		{
			ROS_INFO_STREAM("Initializing absolute pulse values.");
			prev_l_pulses = left_count;
			prev_r_pulses = right_count;
			abs_init = true;
		}
		left_count_abs = left_count;
		right_count_abs = right_count;
		left_count = left_count - prev_l_pulses;
		right_count = right_count - prev_r_pulses;
		prev_l_pulses = left_count_abs;
		prev_r_pulses = right_count_abs;
	}

	/////////////////
	//wrap encoders//
	/////////////////
	if (wrapping_enabled)
	{
		//left encoder
		if ((left_count < encoder_low_wrap) && (prev_l_enc > encoder_high_wrap))
		{
			lmult = lmult + 1;
			left_count = (encoder_max - prev_l_enc) + (left_count - encoder_min);
		}
		if ((left_count > encoder_high_wrap) && (prev_l_enc < encoder_low_wrap))
		{
			lmult = lmult - 1;
			left_count = (encoder_max - left_count) + (prev_l_enc - encoder_min);
		}
		//right encoder
		if ((right_count < encoder_low_wrap) && (prev_r_enc > encoder_high_wrap))
		{
			rmult = rmult + 1;
			right_count = (encoder_max - prev_r_enc) + (right_count - encoder_min);
		}
		if ((right_count > encoder_high_wrap) && (prev_r_enc < encoder_low_wrap))
		{
			rmult = rmult - 1;
			right_count = (encoder_max - right_count) + (prev_r_enc - encoder_min);
		}

		prev_l_enc = left_count;
		prev_r_enc = right_count;
	}
	//////////////////////////////////////
	//calculate current and elapsed time//
	//////////////////////////////////////
	now = ros::Time::now();
	elapsed = now.toSec() - then.toSec();

	/////////////////////////////////////
	//calculate absolute encoder values//
	/////////////////////////////////////
	if (!sub_to_abs)
	{	
		if (wrapping_enabled)
		{
			left_count_abs = 1.0 * (left_count + lmult * (encoder_max - encoder_min));
			right_count_abs = 1.0 * (right_count + rmult * (encoder_max - encoder_min));
		}
		else
		{
			left_count_abs += left_count;
			right_count_abs += right_count;		
		}
	}

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
		DeltaTh = (d_right - d_left) / track_width; // th is tan(th)
		dr = DeltaTh / elapsed;
		theta_final = theta_final + DeltaTh;
	}
	else
	{
		ROS_INFO_STREAM("using imu88888888888888888888888888888888888888888888888888888888");
		theta_final = imu_yaw;
    	dr = (imu_yaw - prev_imu_yaw)/elapsed;
	}

	////////////////////////
	//update previous time//
	////////////////////////
	then = now;

	////////////////
	//debug prints//
	////////////////
	ROS_INFO_STREAM("x final: " << x_final);
	ROS_INFO_STREAM("y final: " << y_final);
	ROS_INFO_STREAM("theta_final: " << theta_final*180.0/3.12);
	ROS_INFO_STREAM("left_abs_hall_count: " << left_count_abs);
	ROS_INFO_STREAM("right_abs_hall_count: " << right_count_abs);

}

void Odometry_calc::TfPub()
{
	/////////////////////////////////////
	//set up and publish transformation//
	/////////////////////////////////////
	geometry_msgs::TransformStamped odom_trans;
	odom_quat = tf::createQuaternionMsgFromYaw(theta_final);
	odom_trans.header.stamp = now;
	odom_trans.header.frame_id = tf_header_frame; //originally was set to odom
	odom_trans.child_frame_id = tf_child_frame;
	odom_trans.transform.translation.x = x_final;
	odom_trans.transform.translation.y = y_final;
	odom_trans.transform.translation.z = 0.0;
	odom_trans.transform.rotation = odom_quat;
	odom_broadcaster.sendTransform(odom_trans);
}

void Odometry_calc::OdomPub()
{
	///////////////////////////////
	//set up and publish odometry//
	///////////////////////////////
	nav_msgs::Odometry odom;
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
	odom_pub.publish(odom);
}

//Spin function
void Odometry_calc::spin()
{
	ros::Rate loop_rate(rate);
	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}
}

//Encoder callback
void Odometry_calc::encoderBCR(const roboteq_motor_controller_driver::channel_values &ticks)

{
	right_count = ticks.value[0];
	left_count = ticks.value[1];
	update();
	if (tf_publish)
	{
		TfPub();
	}
	OdomPub();
}

void Odometry_calc::imu_setup()
{
  if (enable_imu_yaw)
  {
    imu_sub = n.subscribe(imu_topic, 100, &Odometry_calc::imu_callback, this);
  };
}

void Odometry_calc::imu_callback(const sensor_msgs::Imu &imu)
{
	prev_imu_yaw = imu_yaw;
  //TF quaternion
  tf::Quaternion q(
      imu.orientation.x,
      imu.orientation.y,
      imu.orientation.z,
      imu.orientation.w);

  // TF matrix
  tf::Matrix3x3 m(q);
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
        imu_yaw_init /= (double)(imu_cnt);
      }
    }
    else
    {
      imu_yaw = (double)yaw - (double)imu_yaw_init;
    }
  }
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "diff_odom");
	ros::NodeHandle n;
	Odometry_calc obj(n);
	obj.spin();
	return 0;
}