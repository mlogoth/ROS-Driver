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

	int channel_number_1;
	int channel_number_2;
	int frequencyH;
	int frequencyL;
	int frequencyG;

	int motor_1_accel;
	int motor_2_accel;
	int motor_1_dec;
	int motor_2_dec;
	int motor_1_alim;
	int motor_2_alim;
	int motor_1_maxrpm;
	int motor_2_maxrpm;
	int motor_1_initabscount;
	int motor_2_initabscount;
	int configs_init ; 

	ros::NodeHandle nh;

	void initialize()
	{

		nh.getParam("port", port);
		nh.getParam("baud", baud);
		nh.getParam("track_width", track_width);
		nh.getParam("reduction_ratio", reduction_ratio);
		nh.getParam("max_vel", max_vel);
		nh.getParam("wheel_circumference", wheel_circumference);

		//read parameters-limits to configure the driver
		nh.param("motor_1_accel", motor_1_accel, 200);
		nh.param("motor_2_accel", motor_2_accel, 200);
		nh.param("motor_1_dec", motor_1_dec, 200);
		nh.param("motor_2_dec", motor_2_dec, 200);
		nh.param("motor_1_alim", motor_1_alim, 4);
		nh.param("motor_2_alim", motor_2_alim, 4);
		nh.param("motor_1_maxrpm", motor_1_maxrpm, 1000);
		nh.param("motor_2_maxrpm", motor_2_maxrpm, 1000);
		nh.param("motor_1_initabscount", motor_1_initabscount, 0);
		nh.param("motor_2_initabscount", motor_2_initabscount, 0);
		ROS_INFO_STREAM("motor_1_initabscount : " << motor_1_initabscount);
		cmd_vel_sub = nh.subscribe("/cmd_vel", 10, &RoboteqDriver::cmd_vel_callback, this);

		configs_init = 0 ; 
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

	void cmd_vel_callback(const geometry_msgs::Twist &msg)
	{	 
		// wheel speed (m/s)
		double right_speed = msg.linear.x - track_width * msg.angular.z / 2.0;
		double left_speed = msg.linear.x + track_width * msg.angular.z / 2.0;

		// ROS_INFO_STREAM("================================");
		// ROS_INFO_STREAM("right_speed:" << right_speed);
		// ROS_INFO_STREAM("left_speed" << left_speed);

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

		int32_t right_rpm = (right_speed * reduction_ratio * 60) / (wheel_circumference);
    	int32_t left_rpm = (left_speed * reduction_ratio * 60) / (wheel_circumference);

		std::stringstream right_cmd;
		std::stringstream left_cmd;

		right_cmd << "!S 1 " << (int)(right_rpm) << "\r";
		left_cmd << "!S 2 " << (int)(left_rpm) << "\r";

		ROS_INFO_STREAM("----------------------------");
		ROS_INFO_STREAM("Wheel Motors: right_rpm: "<<right_rpm << " - left_rpm: "<<left_rpm);
		
		ser.write(right_cmd.str());
		ser.write(left_cmd.str());
		ser.flush();
	}

	ros::NodeHandle n;
	ros::NodeHandle privn;
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
		ROS_INFO_STREAM("********************************************************************************");
		std::stringstream str;
		str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
		ser.write(str.str());
		ser.flush();
		response.result = str.str();

		//ROS_INFO_STREAM(response.result);
		ROS_INFO("********************************************************************************");
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

		// privn = ros::NodeHandle("~");
		// ros::ServiceClient config_client = privn.serviceClient<roboteq_motor_controller_driver::config_srv>("config_service");
		// ros::ServiceClient command_client = privn.serviceClient<roboteq_motor_controller_driver::command_srv>("command_service");

		// roboteq_motor_controller_driver::config_srv config_s;
		// roboteq_motor_controller_driver::command_srv command_s;

		// command_s.request.userInput = "CB";
		// command_s.request.channel = 1;
		// command_s.request.value = motor_1_initabscount;
		// if (command_client.call(command_s))
		// {
		// 	ROS_INFO_STREAM("success");
		// }
		// else
		// {
		// 	ROS_INFO_STREAM("failure");
		// 	ROS_INFO_STREAM(command_client.call(command_s));
		// }
	}

	void set_limits()
	{
		privn = ros::NodeHandle("~");
		ros::ServiceClient config_client = privn.serviceClient<roboteq_motor_controller_driver::config_srv>("config_service");
		ros::ServiceClient command_client = privn.serviceClient<roboteq_motor_controller_driver::command_srv>("command_service");

		roboteq_motor_controller_driver::config_srv config_s;
		roboteq_motor_controller_driver::command_srv command_s;


		// //Motor 1
		// config_s.request.userInput = "ALIM";
		// config_s.request.channel = 1;
		// config_s.request.value = motor_1_alim;
		// //config_client.call(config_s);

		// config_s.request.userInput = "MXRPM";
		// config_s.request.channel = 1;
		// config_s.request.value = motor_1_maxrpm;
		// //config_client.call(config_s);

		// command_s.request.userInput = "AC";
		// command_s.request.channel = 1;
		// command_s.request.value = motor_1_accel;
		// //command_client.call(command_s);

		// command_s.request.userInput = "DC";
		// command_s.request.channel = 1;
		// command_s.request.value = motor_1_dec;
		// //command_client.call(command_s);

		command_s.request.userInput = "CB";
		command_s.request.channel = 1;
		command_s.request.value = motor_1_initabscount;
		if (command_client.call(command_s))
		{
			ROS_INFO_STREAM("success");
		}
		else
		{
			ROS_INFO_STREAM("failure");
			ROS_INFO_STREAM(command_client.call(command_s));
		}

		//Motor 2
		config_s.request.userInput = "ALIM";
		config_s.request.channel = 2;
		config_s.request.value = motor_2_alim;
		//config_client.call(config_s);

		config_s.request.userInput = "MXRPM";
		config_s.request.channel = 2;
		config_s.request.value = motor_2_maxrpm;
		//config_client.call(config_s);

		command_s.request.userInput = "AC";
		command_s.request.channel = 2;
		command_s.request.value = motor_2_accel;
		//command_client.call(command_s);

		command_s.request.userInput = "DC";
		command_s.request.channel = 2;
		command_s.request.value = motor_2_dec;
		//command_client.call(command_s);

		command_s.request.userInput = "CB";
		command_s.request.channel = 2;
		command_s.request.value = motor_2_initabscount;
		// if (command_client.call(command_s))
		// {
		// 	ROS_INFO_STREAM("success");
		// }
		// else
		// {
		// 	ROS_INFO_STREAM("failure");
		// }
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



		ros::Rate loop_rate(5);
		while (ros::ok())
		{

			ros::spinOnce();

			if (configs_init == 0){
				set_limits();
				configs_init +=  1 ; 
			}
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
	ros::init(argc, argv, "roboteq_motor_controller_driver_wheel");

	RoboteqDriver driver;

	ros::waitForShutdown();

	return 0;
}
