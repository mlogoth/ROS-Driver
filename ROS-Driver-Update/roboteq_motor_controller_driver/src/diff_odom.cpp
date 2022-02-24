#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int16.h"
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>

#define NORMALIZE(_z) atan2(sin(_z), cos(_z))

class Odometry_calc
{

public:
	Odometry_calc();

	void spin();

private:
	ros::NodeHandle n;
	ros::Subscriber sub;
	ros::Subscriber wheel_sub;
	// ros::Subscriber r_wheel_sub;
	ros::Publisher odom_pub;

	tf::TransformBroadcaster odom_broadcaster;
	//Encoder related variables
	double encoder_min;
	double encoder_max;

	double encoder_low_wrap;
	double encoder_high_wrap;

	double prev_lencoder;
	double prev_rencoder;

	double left_abs;
	double right_abs;

	double lmult;
	double rmult;

	double left;
	double right;

	// double abs_left;
	// double abs_right;

	// double prev_left;
	// double prev_right;

	double rate;

	ros::Duration t_delta;

	ros::Time t_next;

	ros::Time then;

	double enc_left;

	double enc_right;

	double ticks_meter;

	double encoder_cpr;

	double base_width;

	double S;

	double wheel_circumference;

	double dx;
	double meters_per_tick;

	double dr;

	double x_final, y_final, theta_final, odom_yaw;

	ros::Time current_time, last_time;

	void encoderBCR(const roboteq_motor_controller_driver::channel_values &left_ticks);

	void init_variables();

	void update();
};

Odometry_calc::Odometry_calc()
{

	init_variables();

	ROS_INFO("Started odometry computing node");

	wheel_sub = n.subscribe("/track_motor_controller/hall_count", 1, &Odometry_calc::encoderBCR, this);
	// wheel_sub = n.subscribe("/hall_count", 1, &Odometry_calc::encoderBCR, this);

	odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);

	//Retrieving parameters of this node
}

void Odometry_calc::init_variables()
{

	prev_lencoder = 0;
	prev_rencoder = 0;

	left_abs = 0;
	right_abs = 0;

	lmult = 0;
	rmult = 0;

	left = 0;
	right = 0;

	encoder_min = -65536;
	encoder_max = 65536;

	rate = 10;

	ticks_meter = 50;

	base_width = 0.91;

	encoder_cpr = 4368;
	wheel_circumference = 1.2854;

	encoder_low_wrap = ((encoder_max - encoder_min) * 0.3) + encoder_min;
	encoder_high_wrap = ((encoder_max - encoder_min) * 0.7) + encoder_min;

	t_delta = ros::Duration(1.0 / rate);
	t_next = ros::Time::now() + t_delta;

	then = ros::Time::now();

	enc_left = 10;
	enc_right = 0;

	dx = 0;
	dr = 0;

	x_final = 0;
	odom_yaw = 0.0;
	y_final = 0;
	theta_final = 0;

	S = 0;

	current_time = ros::Time::now();
	last_time = ros::Time::now();

	double gearTeeth = 18;
	double beltPerimeter = 1.2854;
	double hallRes = 12;
	double beltTeeth = 63;
	double reduction = 91;

	meters_per_tick = (gearTeeth * beltPerimeter) / (hallRes * beltTeeth * reduction);

	// prev_left = 0;
	// prev_right = 0;
}

//Spin function
void Odometry_calc::spin()
{

	ros::Rate loop_rate(rate);

	while (ros::ok())
	{
		// update();
		ros::spinOnce();
		loop_rate.sleep();
	}
}

//Update function
void Odometry_calc::update()
{

	ros::Time now = ros::Time::now();

	//	ros::Time elapsed;

	double elapsed;

	double d_left, d_right, DeltaS, DeltaTh, DeltaX, DeltaY;

	if (now > t_next)
	{

		elapsed = now.toSec() - then.toSec();

		// 	        ROS_INFO_STREAM("elapsed =" << elapsed);

		if (enc_left == 0)
		{
			d_left = 0;
			d_right = 0;
		}
		else
		{
			// left_abs += left;
			// ROS_INFO_STREAM("Left " << left_abs << "  right " << right_abs);
			d_left = -left * meters_per_tick;
			// ROS_INFO_STREAM("Left___" << left);
			d_right = -right * meters_per_tick;
		}

		DeltaS = (d_left + d_right) / 2.0;
		DeltaTh = (d_right - d_left) / (base_width); // th is tan(th)

		dx = DeltaS / elapsed;
		dr = DeltaTh / elapsed;

		if (DeltaS != 0)
		{
			DeltaX = DeltaS * cos(theta_final + (DeltaTh / 2));
			DeltaY = DeltaS * sin(theta_final + (DeltaTh / 2));

			x_final = x_final + DeltaX;
			y_final = y_final + DeltaY;
		}
		// ROS_INFO("x: %d",x_final);
		// ROS_INFO("y: %d",y_final);

		if (DeltaTh != 0)
			theta_final = theta_final + DeltaTh;
		// ROS_INFO_STREAM(theta_final);
		// geometry_msgs::Quaternion odom_quat;
		geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta_final);

		//first, we'll publish the transform over tf
		geometry_msgs::TransformStamped odom_trans;
		odom_trans.header.stamp = now;
		odom_trans.header.frame_id = "map"; //originally was set to odom
		odom_trans.child_frame_id = "base_footprint";

		odom_trans.transform.translation.x = x_final;
		odom_trans.transform.translation.y = y_final;
		odom_trans.transform.translation.z = 0.0;
		odom_trans.transform.rotation = odom_quat;

		//send the transform
		odom_broadcaster.sendTransform(odom_trans);

		//next, we'll publish the odometry message over ROS
		nav_msgs::Odometry odom;
		odom.header.stamp = now;
		odom.header.frame_id = "odom";

		//set the position
		odom.pose.pose.position.x = x_final;
		odom.pose.pose.position.y = y_final;
		odom.pose.pose.position.z = 0.0;
		odom.pose.pose.orientation = odom_quat;

		//set the velocity
		odom.child_frame_id = "base_footprint";
		odom.twist.twist.linear.x = dx;
		odom.twist.twist.linear.y = 0;
		odom.twist.twist.angular.z = dr;

		//publish the message
		odom_pub.publish(odom);

		then = now;

		left = 0;
		right = 0;
	}
}

//Left encoder callback
void Odometry_calc::encoderBCR(const roboteq_motor_controller_driver::channel_values &ticks)

{	
	right = ticks.value[0];
	left = ticks.value[1];

	// abs_right = ticks.value[0];
	// abs_left = ticks.value[1];

	// right = abs_right - prev_right;
	// left = abs_left - prev_left;

	update();

	// prev_right = abs_right;
	// prev_left = abs_left;

	// ROS_INFO_STREAM("Left:" << right << " Right: " << left);
}


int main(int argc, char **argv)

{
	ros::init(argc, argv, "diff_odom");
	Odometry_calc obj;
	obj.spin();

	return 0;
}
