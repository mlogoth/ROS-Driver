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
#include <cassert>
#include <mutex>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <roboteq_motor_controller_driver/querylist.h>
#include <roboteq_motor_controller_driver/channel_values.h>
#include <roboteq_motor_controller_driver/config_srv.h>
#include <roboteq_motor_controller_driver/command_srv.h>
#include <roboteq_motor_controller_driver/maintenance_srv.h>



static const std::string tag {"[RoboteQ] "};


class RoboteqDriver
{
public:
	RoboteqDriver( ros::NodeHandle nh, ros::NodeHandle nh_priv );


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


	ros::NodeHandle nh;

//added in changes
	ros::NodeHandle nh_priv_;
	std::vector<int> f_list;
	std::vector<ros::Publisher> query_pub_;
	ros::NodeHandle nh_;
	ros::Timer timer_pub_;
	serial::Serial ser_;
	std::mutex 	locker;
	ros::Publisher serial_read_pub_;

	ros::NodeHandle n;
	ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer maintenancesrv;

	std::vector<int> cum_query_size;


	void queryCallback	(const ros::TimerEvent &);

	void formQuery(std::string, 
				std::map<std::string,std::string> &, 
				std::vector<ros::Publisher> &,
				std::stringstream &);

	void initialize()
	{

		nh_.getParam("port", port);
		nh_.getParam("baud", baud);

		if (!nh_.getParam("rate", rate))
		{
			rate = 5; 
		}

		if (!nh_.getParam("channel_mode", channel_mode))
		{
			ROS_INFO_STREAM("No channel mode was selected. Assigning driver as dual.");
			channel_mode = "dual";
		}
		if ((channel_mode ==   "dual") && (!nh_.getParam("motor_type", motor_type)))
		{
			ROS_INFO_STREAM("No valid general motor type was selected. Assigning skid_steering.");
			motor_type = "skid_steering";
		}
		if ((channel_mode == "single") && (!nh_.getParam("motor_1_type", motor_1_type)))
		{
			ROS_ERROR_STREAM("No motor type was found for motor 1. Shutting down.");
			ros::shutdown();
		}
		if ((channel_mode == "single") && (!nh_.getParam("motor_2_type", motor_2_type)))
		{
			ROS_ERROR_STREAM("No motor type was found for motor 2. Shutting down.");
			ros::shutdown();
		}

		if (motor_type == "skid_steering")
		{
			nh_.getParam("track_width", track_width);
			nh_.getParam("max_vel", max_vel);
			if (!nh_.getParam("reduction_ratio", reduction_ratio))
			{
				reduction_ratio = 70;
			}
			nh_.getParam("wheel_circumference", wheel_circumference);
			ROS_INFO_STREAM("Driver controls two motors moving a skid steering vehicle.");
			cmd_vel_sub = nh_.subscribe("cmd_vel", 10, &RoboteqDriver::skid_steering_vel_callback, this);
		}
		else if (channel_mode == "dual")
		{
			ROS_INFO_STREAM("Driver controls two motors with a single value.");
			if (motor_1_type == "set_speed")
			{
				ROS_INFO_STREAM("Motor 1 is operating in closed loop mode.");
				cmd_vel_channel_1_sub = nh_.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_1_vel_callback, this);				
			}
			else
			{
				ROS_INFO_STREAM("Motor 1 is operating in open loop mode.");
				cmd_vel_channel_1_sub = nh_.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_1_vel_callback, this);
			}
			if (motor_2_type == "set_speed")
			{
				ROS_INFO_STREAM("Motor 2 is operating in closed loop mode.");
				cmd_vel_channel_2_sub = nh_.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_2_vel_callback, this);				
			}
			else
			{
				ROS_INFO_STREAM("Motor 2 is operating in open loop mode.");
				cmd_vel_channel_2_sub = nh_.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_2_vel_callback, this);
			}
		}
		else if (channel_mode == "single")
		{
			ROS_INFO_STREAM("Driver controls two motors seperately.");
			if (motor_1_type == "set_speed")
			{
				ROS_INFO_STREAM("Motor 1 is operating in closed loop mode.");
				cmd_vel_channel_1_sub = nh_.subscribe("chan_1_set_vel", 10, &RoboteqDriver::channel_1_vel_callback, this);				
			}
			else
			{
				ROS_INFO_STREAM("Motor 1 is operating in open loop mode.");
				cmd_vel_channel_1_sub = nh_.subscribe("chan_1_go_to_vel", 10, &RoboteqDriver::channel_1_vel_callback, this);
			}
			if (motor_2_type == "set_speed")
			{
				ROS_INFO_STREAM("Motor 2 is operating in closed loop mode.");
				cmd_vel_channel_2_sub = nh_.subscribe("chan_2_set_vel", 10, &RoboteqDriver::channel_2_vel_callback, this);				
			}
			else
			{
				ROS_INFO_STREAM("Motor 2 is operating in open loop mode.");
				cmd_vel_channel_2_sub = nh_.subscribe("chan_2_go_to_vel", 10, &RoboteqDriver::channel_2_vel_callback, this);
			}
		}


		connect();
	}

	void connect()
	{

		try
		{

			ser_.setPort(port);
			ser_.setBaudrate(baud); //get baud as param
			serial::Timeout to = serial::Timeout::simpleTimeout(10);
			ser_.setTimeout(to);
			ser_.open();
		}
		catch (serial::IOException &e)
		{

			ROS_ERROR_STREAM("Unable to open port ");
			ROS_INFO_STREAM("Unable to open port");
			;
		}
		if (ser_.isOpen())
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
		//ROS_INFO_STREAM("Wheel Motors: right_rpm: "<<right_rpm << " - left_rpm: "<<left_rpm);
		
		ser_.write(right_cmd.str());
		ser_.write(left_cmd.str());
		ser_.flush();
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

		ser_.write(right_cmd.str());
		ser_.write(left_cmd.str());
		ser_.flush();
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

		ser_.write(channel_1_cmd.str());
		ser_.flush();
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

		ser_.write(channel_2_cmd.str());
		ser_.flush();
	}

	bool configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response)
	{
		std::stringstream str;
		str << "^" << request.userInput << " " << request.channel << " " << request.value << "_ "
			<< "%\clsav321654987";
		ser_.write(str.str());
		ser_.flush();
		response.result = str.str();

		ROS_INFO_STREAM(response.result);
		return true;
	}

	bool commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response)
	{
		std::stringstream str;
		str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
		ser_.write(str.str());
		ser_.flush();
		response.result = str.str();

		ROS_INFO_STREAM(response.result);
		return true;
	}

	bool maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response)
	{
		std::stringstream str;
		str << "%" << request.userInput << " "
			<< "_";
		ser_.write(str.str());
		ser_.flush();
		response.result = ser_.read(ser_.available());

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
		
		nh_.getParam("frequency_list", f_list);	

		
		std::stringstream ss_gen;
		ss_gen << "^echof 1_"; // Disable echo from driver
		ss_gen << "# c_"; // Clear Buffer History of previous queries

		for (int i = 0; i < f_list.size(); i++)
		{		
			if (f_list[i] > 0)
			{
				ss_gen << "/\"DF" << i << "?\",\"?\"";
				std::map<std::string, std::string> query_map;
				std::stringstream query_name;
				query_name << "query" << i;
				formQuery(query_name.str(), query_map, query_pub_, ss_gen);
				ss_gen << "# " << 1000/f_list[i] << "_";
			}
			else
			{
				ROS_ERROR_STREAM(tag << "Negative frequency detected "<< f_list[i]);
			}			
		}
		
		ser_.write(ss_gen.str()); // Send commands and queries
		ser_.flush();
		serial_read_pub_ = nh_.advertise<std_msgs::String>("read_serial", 1000);
		
		int max_freq;
		max_freq = *std::max_element(f_list.begin(), f_list.end());
		timer_pub_ = nh_.createTimer(ros::Duration(1/max_freq), &RoboteqDriver::queryCallback, this);
	}
};

RoboteqDriver::RoboteqDriver(ros::NodeHandle nh, ros::NodeHandle nh_priv):
	nh_(nh),
	nh_priv_(nh_priv)
	{
		initialize();
	}

void RoboteqDriver::queryCallback(const ros::TimerEvent &){
	std_msgs::String result;

	// Mutex is probably not needed in single threaded spinning
	std::lock_guard<std::mutex> lock(locker); // Lock mutex until end of scope

	if (ser_.available()){
		result.data = ser_.read(ser_.available()); // Read all available bytes

		serial_read_pub_.publish(result); // Publish raw data
		
		std::vector<std::string> fields;
		
		boost::split(fields, result.data, boost::algorithm::is_any_of("\r")); // Split by termination character "\r"
		
		if (fields.size() < 2){
			ROS_ERROR_STREAM(tag << "Empty data:{" << result.data << "}");
			return;
		}
		
		std::vector<std::string> query_fields;
		std::vector<std::string> sub_query_fields;
		int frequency_index;			
		
		for (int i = 0; i < fields.size() - 1; i++){
			if (fields[i].rfind("DF", 0) == 0) // if field starts with "DF"
			{
				frequency_index = boost::lexical_cast<int>(fields[i][2]);
				try{
					query_fields.clear();
					boost::split(query_fields, fields[i], boost::algorithm::is_any_of("?"));											
					for (int j = 1; j < query_fields.size(); j++){					
						sub_query_fields.clear();
						boost::split(sub_query_fields, query_fields[j], boost::algorithm::is_any_of(":"));
						
						roboteq_motor_controller_driver::channel_values msg;
						for (int k = 0; k < sub_query_fields.size(); k++){
							try{
								msg.value.push_back(boost::lexical_cast<int>(sub_query_fields[k]));
							}
							catch (const std::exception &e){

								ROS_ERROR_STREAM(tag << "Garbage data on Serial " << fields[i] << "//" << query_fields[j] << "//" << sub_query_fields[k]);
								std::cerr << e.what() << '\n';
								break;
							}
						}
						query_pub_[cum_query_size[frequency_index] + j-1].publish(msg);
					}						
					
				}
				catch (const std::exception &e){
					std::cerr << e.what() << '\n';
					ROS_ERROR_STREAM(tag << "Finding query output in :" << fields[i]);
					continue;
				}
			}			
		}		
	}
}

void RoboteqDriver::formQuery(std::string param, 
							std::map<std::string,std::string> &queries, 
							std::vector<ros::Publisher> &pubs,
							std::stringstream &ser_str){
	nh_.getParam(param, queries);
	int count = 0;
	for (std::map<std::string, std::string>::iterator iter = queries.begin(); iter != queries.end(); iter++){
		ROS_INFO_STREAM(tag << param  <<" Publish topic: " << iter->first);
		pubs.push_back(nh_.advertise<roboteq_motor_controller_driver::channel_values>(iter->first, 100));

		std::string cmd = iter->second;
		ser_str << cmd << "_";
		count++;
	}
	if (!cum_query_size.empty())
	{
		cum_query_size.push_back(count + cum_query_size.back());
	}
	else
	{
		cum_query_size.push_back(0);
		cum_query_size.push_back(count);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_motor_controller_driver");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	RoboteqDriver driver(nh, nh_priv);

	//ros::MultiThreadedSpinner spinner(8);
	//spinner.spin();
	ros::spin(); // multithreading doesn't provide any benefit cause of mutex in callback
	ros::waitForShutdown();

	return 0;
}
