// #include <roboteq_motor_controller_driver/roboteq_motor_controller_driver_node.h>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>

#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <roboteq_motor_controller_driver/querylist.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <roboteq_motor_controller_driver/channel_values.h>
#include <roboteq_motor_controller_driver/config_srv.h>
#include <roboteq_motor_controller_driver/command_srv.h>
#include <roboteq_motor_controller_driver/maintenance_srv.h>



class RoboteqDriver
{
public:
	RoboteqDriver()
	{
		initialize(); //constructor - Initialize
	}

	~RoboteqDriver()
	{
		if (ser.isOpen())
		{
			ser.close();
		}
	}

private:
	serial::Serial ser;
	std::string port;
	int32_t baud;
	double wheel_circumference;
	double track_width;
	double max_vel;
	int max_rpm;
	double reduction_ratio;
	ros::Publisher read_publisher;
	ros::Subscriber cmd_vel_sub;

	std::string motor_type;
	std::string motor_1_type;
	std::string motor_2_type;	
	std::string channel_mode;
	ros::Subscriber cmd_vel_channel_1_sub;
	ros::Subscriber cmd_vel_channel_2_sub;
	int rate; 

	int channel_number_1;
	int channel_number_2;
	int frequencyH;
	int frequencyL;
	int frequencyG;
	ros::NodeHandle nh;

	void initialize()
	{

		nh.getParam("port", port);
		nh.getParam("baud", baud);

		if (!nh.getParam("rate", rate))
		{
			rate = 20; 
		}

		if (!nh.getParam("channel_mode", channel_mode))
		{
			ROS_INFO_STREAM("No channel mode was selected. Assigning driver as dual.");
			channel_mode = "dual";
		}
		if ((channel_mode ==   "dual") && (!nh.getParam("motor_type", motor_type)))
		{
			ROS_INFO_STREAM("No valid general motor type was selected. Assigning skid_steering.");
			motor_type = "skid_steering";
		}
		if ((channel_mode == "single") && (!nh.getParam("motor_1_type", motor_1_type)))
		{
			ROS_ERROR_STREAM("No motor type was found for motor 1. Shutting down.");
			ros::shutdown();
		}
		if ((channel_mode == "single") && (!nh.getParam("motor_2_type", motor_2_type)))
		{
			ROS_ERROR_STREAM("No motor type was found for motor 2. Shutting down.");
			ros::shutdown();
		}

		if (motor_type == "skid_steering")
		{
			nh.getParam("track_width", track_width);
			nh.getParam("max_vel", max_vel);
			if (!nh.getParam("reduction_ratio", reduction_ratio))
			{
				reduction_ratio = 70;
			}
			nh.getParam("wheel_circumference", wheel_circumference);
			ROS_INFO_STREAM("Driver controls two motors moving a skid steering vehicle.");
			cmd_vel_sub = nh.subscribe("cmd_vel", 10, &RoboteqDriver::skid_steering_vel_callback, this);
		}
		else if (channel_mode == "dual")
		{
			ROS_INFO_STREAM("Driver controls two motors with a single value.");
			if (motor_1_type == "set_speed")
			{
				ROS_INFO_STREAM("Motor 1 is operating in closed loop mode.");
				cmd_vel_channel_1_sub = nh.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_1_vel_callback, this);				
			}
			else
			{
				ROS_INFO_STREAM("Motor 1 is operating in open loop mode.");
				cmd_vel_channel_1_sub = nh.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_1_vel_callback, this);
			}
			if (motor_2_type == "set_speed")
			{
				ROS_INFO_STREAM("Motor 2 is operating in closed loop mode.");
				cmd_vel_channel_2_sub = nh.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_2_vel_callback, this);				
			}
			else
			{
				ROS_INFO_STREAM("Motor 2 is operating in open loop mode.");
				cmd_vel_channel_2_sub = nh.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_2_vel_callback, this);
			}
		}
		else if (channel_mode == "single")
		{
			ROS_INFO_STREAM("Driver controls two motors seperately.");
			if (motor_1_type == "set_speed")
			{
				ROS_INFO_STREAM("Motor 1 is operating in closed loop mode.");
				cmd_vel_channel_1_sub = nh.subscribe("chan_1_set_vel", 10, &RoboteqDriver::channel_1_vel_callback, this);				
			}
			else
			{
				ROS_INFO_STREAM("Motor 1 is operating in open loop mode.");
				cmd_vel_channel_1_sub = nh.subscribe("chan_1_go_to_vel", 10, &RoboteqDriver::channel_1_vel_callback, this);
			}
			if (motor_2_type == "set_speed")
			{
				ROS_INFO_STREAM("Motor 2 is operating in closed loop mode.");
				cmd_vel_channel_2_sub = nh.subscribe("chan_2_set_vel", 10, &RoboteqDriver::channel_2_vel_callback, this);				
			}
			else
			{
				ROS_INFO_STREAM("Motor 2 is operating in open loop mode.");
				cmd_vel_channel_2_sub = nh.subscribe("chan_2_go_to_vel", 10, &RoboteqDriver::channel_2_vel_callback, this);
			}
		}


		connect();
	}

	void connect()
	{

		try
		{

			ser.setPort(port);
			ser.setBaudrate(baud); //get baud as param
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);
			ser.open();
		}
		catch (serial::IOException &e)
		{

			ROS_ERROR_STREAM("Unable to open port ");
			ROS_INFO_STREAM("Unable to open port");
			;
		}
		if (ser.isOpen())
		{

			ROS_INFO_STREAM("Serial Port initialized\"");
		}
		else
		{
			// ROS_INFO_STREAM("HI4");
			ROS_INFO_STREAM("Serial Port is not open");
		}
		run();
	}

	void skid_steering_vel_callback(const geometry_msgs::Twist &msg)
	{	 
		// wheel speed (m/s)
		double right_speed = msg.linear.x - track_width * msg.angular.z / 2.0;
		double left_speed = msg.linear.x + track_width * msg.angular.z / 2.0;

		// ROS_INFO_STREAM("================================");
		// ROS_INFO_STREAM("right_speed: " << right_speed);
		// ROS_INFO_STREAM("left_speed: " << left_speed);

		// set maximum linear speed at 0.35 m/s (4500rpm in motor)
		if (fabs(right_speed) > max_vel)
		{
			if (right_speed > 0) {right_speed = max_vel;}
			else {right_speed = -max_vel;}
		}
		if (fabs(left_speed) > max_vel)
		{
			if (left_speed > 0) {left_speed = max_vel;}
			else {left_speed = -max_vel;}
		}
		// std::stringstream cmd_sub;
		// ROS_INFO_STREAM("================================");
		// ROS_INFO_STREAM("right_speed: " << right_speed);
		// ROS_INFO_STREAM("left_speed: " << left_speed);

		int32_t right_rpm = (right_speed * reduction_ratio * 60) / (wheel_circumference);
    	int32_t left_rpm = (left_speed * reduction_ratio * 60) / (wheel_circumference);

		// ROS_INFO_STREAM(reduction_ratio);
		// ROS_INFO_STREAM("================================");
		// ROS_INFO_STREAM("right_rpm: " << right_rpm);
		// ROS_INFO_STREAM("left_rpm: " << left_rpm);

		std::stringstream right_cmd;
		std::stringstream left_cmd;

		right_cmd << "!S 1 " << (int)(right_rpm) << "\r";
		left_cmd << "!S 2 " << (int)(left_rpm) << "\r";

		// ROS_INFO_STREAM("----------------------------");
		ROS_INFO_STREAM("Wheel Motors: right_rpm: "<<right_rpm << " - left_rpm: "<<left_rpm);
		
		ser.write(right_cmd.str());
		ser.write(left_cmd.str());
		ser.flush();
	}

	void dual_vel_callback(const std_msgs::Int16 &msg)
	{	 
		// wheel speed (m/s)
		int cmd = msg.data;

		// ROS_INFO_STREAM("================================");
		// ROS_INFO_STREAM("Motor Command:" << cmd);

		std::stringstream right_cmd;
		std::stringstream left_cmd;

		if (motor_type == "go_to_speed")
		{
			right_cmd << "!G 2 " << cmd << "\r";
			left_cmd << "!G 1 " << cmd << "\r";			
		}
		else if (motor_type == "set_speed")
		{
			right_cmd << "!S 2 " << cmd << "\r";
			left_cmd << "!S 1 " << cmd << "\r";			
		}

		ser.write(right_cmd.str());
		ser.write(left_cmd.str());
		ser.flush();
	}

	void channel_1_vel_callback(const std_msgs::Int16 &msg)
	{	 
		// wheel speed (m/s)
		int cmd = msg.data;

		std::stringstream channel_1_cmd;

		if (motor_1_type == "go_to_speed")
		{
			channel_1_cmd << "!G 1 " << cmd << "\r";			
		}
		else if (motor_1_type == "set_speed")
		{
			channel_1_cmd << "!S 1 " << cmd << "\r";			
		}

		ser.write(channel_1_cmd.str());
		ser.flush();
	}

	void channel_2_vel_callback(const std_msgs::Int16 &msg)
	{	 
		// wheel speed (m/s)
		int cmd = msg.data;

		std::stringstream channel_2_cmd;

		if (motor_2_type == "go_to_speed")
		{
			channel_2_cmd << "!G 2 " << cmd << "\r";			
		}
		else if (motor_2_type == "set_speed")
		{
			channel_2_cmd << "!S 2 " << cmd << "\r";			
		}

		ser.write(channel_2_cmd.str());
		ser.flush();
	}


	ros::NodeHandle n;
	ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer maintenancesrv;

	bool configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response)
	{
		std::stringstream str;
		str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
			<< "%\clsav321654987";
		ser.write(str.str());
		ser.flush();
		response.result = str.str();

		ROS_INFO_STREAM(response.result);
		return true;
	}

	bool commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response)
	{
		std::stringstream str;
		str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
		ser.write(str.str());
		ser.flush();
		response.result = str.str();

		ROS_INFO_STREAM(response.result);
		return true;
	}

	bool maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response)
	{
		std::stringstream str;
		str << "%" << request.userInput << " "
			<< "_";
		ser.write(str.str());
		ser.flush();
		response.result = ser.read(ser.available());

		ROS_INFO_STREAM(response.result);
		return true;
	}

	void initialize_services()
	{
		n = ros::NodeHandle();
		configsrv = n.advertiseService("config_service", &RoboteqDriver::configservice, this);
		commandsrv = n.advertiseService("command_service", &RoboteqDriver::commandservice, this);
		maintenancesrv = n.advertiseService("maintenance_service", &RoboteqDriver::maintenanceservice, this);
	}

	void run()
	{
		initialize_services();
		std_msgs::String str1;
		ros::NodeHandle nh;
		nh.getParam("frequencyH", frequencyH);
		nh.getParam("frequencyL", frequencyL);
		nh.getParam("frequencyG", frequencyG);

		typedef std::string Key;
		typedef std::string Val;
		std::map<Key, Val> map_sH;
		nh.getParam("queryH", map_sH);

		std::stringstream ss0;
		std::stringstream ss1;
		std::stringstream ss2;
		std::stringstream ss3;
		std::vector<std::string> KH_vector;

		ss0 << "^echof 1_";
		ss1 << "# c_/\"DH?\",\"?\"";
		for (std::map<Key, Val>::iterator iter = map_sH.begin(); iter != map_sH.end(); ++iter)
		{
			Key KH = iter->first;

			KH_vector.push_back(KH);

			Val VH = iter->second;

			ss1 << VH << "_";
		}
		ss1 << "# " << frequencyH << "_";

		std::vector<ros::Publisher> publisherVecH;
		for (int i = 0; i < KH_vector.size(); i++)
		{
			publisherVecH.push_back(nh.advertise<roboteq_motor_controller_driver::channel_values>(KH_vector[i], 100));
		}

		ser.write(ss0.str());
		ser.write(ss1.str());
		ser.write(ss2.str());
		ser.write(ss3.str());

		ser.flush();
		int count = 0;
		read_publisher = nh.advertise<std_msgs::String>("read", 1000);
		sleep(2);
		ros::Rate loop_rate(rate);
		while (ros::ok())
		{

			ros::spinOnce();
			if (ser.available())
			{

				std_msgs::String result;
				result.data = ser.read(ser.available());

				read_publisher.publish(result);
				boost::replace_all(result.data, "\r", "");
				boost::replace_all(result.data, "+", "");

				std::vector<std::string> fields;

				std::vector<std::string> Field9;
				boost::split(fields, result.data, boost::algorithm::is_any_of("D"));

				std::vector<std::string> fields_H;
				boost::split(fields_H, fields[1], boost::algorithm::is_any_of("?"));

				if (fields_H[0] == "H")
				{

					for (int i = 0; i < publisherVecH.size(); ++i)
					{

						std::vector<std::string> sub_fields_H;

						boost::split(sub_fields_H, fields_H[i + 1], boost::algorithm::is_any_of(":"));
						roboteq_motor_controller_driver::channel_values Q1;

						for (int j = 0; j < sub_fields_H.size(); j++)
						{

							try
							{
								Q1.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
							}
							catch (const std::exception &e)
							{
								count++;
								if (count > 10)
								{
									ROS_INFO_STREAM("Garbage data on Serial");
									//std::cerr << e.what() << '\n';
								}
							}
						}

						publisherVecH[i].publish(Q1);
					}
				}
			}
			loop_rate.sleep();
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_motor_controller_driver");

	RoboteqDriver driver;

	ros::waitForShutdown();

	return 0;
}
