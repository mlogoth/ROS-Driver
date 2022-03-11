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

	int channel_number_1;
	int channel_number_2;
	int frequencyH;
	int frequencyL;
	int frequencyG;

	ros::NodeHandle nh;

//added in changes
	ros::NodeHandle nh_priv_;
	int frequency_H_;
	int frequency_L_;
	std::vector<ros::Publisher> query_pub_H_;
	std::vector<ros::Publisher> query_pub_L_;
	ros::NodeHandle nh_;
	ros::Timer timer_pub_H_;
	ros::Timer timer_pub_L_;
	serial::Serial ser_;
	std::mutex 	locker;
	ros::Publisher serial_read_pub_H_;
	ros::Publisher serial_read_pub_L_;


	void queryCallbackH	(const ros::TimerEvent &);
	void queryCallbackL	(const ros::TimerEvent &);

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


	ros::NodeHandle n;
	ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer maintenancesrv;

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
		nh_.getParam("frequencyH", frequency_H_);
		nh_.getParam("frequencyL", frequency_L_);
		
		std::stringstream ss_gen;
		std::stringstream ss0_H, ss1_H;
		if (frequency_H_ > 0){
			
			ss0_H << "^echof 1_";
			ss1_H << "# c_/\"DH?\",\"?\"";
			std::map<std::string, std::string> query_map_H;
			formQuery("queryH", query_map_H, query_pub_H_, ss1_H);

			ss1_H << "# " << frequency_H_ << "_";
			
			//ser_.write(ss0_H.str());
			///ser_.write(ss1_H.str());
			//ser_.flush();
		}
		std::stringstream ss0_L, ss1_L;
		if (frequency_L_ > 0){
			
			ss0_L << "^echof 1_";
			ss1_L << "/\"DL?\",\"?\"";
			std::map<std::string, std::string> query_map_L;
			formQuery("queryL", query_map_L, query_pub_L_, ss1_L);

			ss1_L << "# " << frequency_L_ << "_";
			
			//ser_.write(ss0_L.str());
			//ser_.write(ss1_L.str());
			
		}
		ss_gen << ss0_H.str() << ss1_H.str() << ss0_L.str() << ss1_L.str();
		ser_.write(ss_gen.str());
		ser_.flush();
		serial_read_pub_H_ = nh_.advertise<std_msgs::String>("read_H", 1000);
		
		//ros::Duration(2).sleep();
		if (frequency_H_ > 0){
			timer_pub_H_ = nh_.createTimer(ros::Duration(frequency_H_/ 1000.), &RoboteqDriver::queryCallbackH, this);
		}

	}
};

RoboteqDriver::RoboteqDriver(ros::NodeHandle nh, ros::NodeHandle nh_priv):
	nh_(nh),
	nh_priv_(nh_priv)
	{
		initialize();
	}

void RoboteqDriver::queryCallbackH(const ros::TimerEvent &){
	int count = 0;
	ros::Time current_time = ros::Time::now();
	if (ser_.available()){


		std_msgs::String result;

		std::lock_guard<std::mutex> lock(locker);

		result.data = ser_.read(ser_.available());

		// std::lock_guard<std::mutex> unlock(locker);

		serial_read_pub_H_.publish(result);
		
		boost::replace_all(result.data, "\r", "");
		//boost::replace_all(result.data, "+", "");

		std::vector<std::string> fields;
		
		boost::split(fields, result.data, boost::algorithm::is_any_of("D"));
		if (fields.size() < 2){

			ROS_ERROR_STREAM(tag << "High Empty data:{" << result.data << "}");
		}
		else if (fields.size() >= 2){
			std::vector<std::string> fields_H;
			std::vector<std::string> fields_L;
			std::vector<std::string> sub_fields_H;
			std::vector<std::string> sub_fields_L;			
			for (int i = fields.size() - 1; i >= 0; i--){
				if (fields[i][0] == 'H')
				{
					try{
						fields_H.clear();
						boost::split(fields_H, fields[i], boost::algorithm::is_any_of("?"));
						if ( fields_H.size() >= query_pub_H_.size() + 1){
							//break;
							if (fields_H.size() > 0){
								for (int gg = 0; gg < fields_H.size()-1; ++gg){
							
									sub_fields_H.clear();
									boost::split(sub_fields_H, fields_H[gg + 1], boost::algorithm::is_any_of(":"));
									
									roboteq_motor_controller_driver::channel_values msg;
									//msg.header.stamp = current_time;

									for (int j = 0; j < sub_fields_H.size(); j++){
										try{
											msg.value.push_back(boost::lexical_cast<int>(sub_fields_H[j]));
										}
										catch (const std::exception &e){

											ROS_ERROR_STREAM(tag << "High Garbage data on Serial " << fields[i] << " " << fields_H[gg + 1] << " "<< sub_fields_H[j]);
											std::cerr << e.what() << '\n';
											break;
										}
									}
									query_pub_H_[gg].publish(msg);
								}
							}
						}
					}
					catch (const std::exception &e){
						std::cerr << e.what() << '\n';
						ROS_ERROR_STREAM(tag << "Finding query output in :" << fields[i]);
						continue;
					}
				}
				else if (fields[i][0] == 'L'){
					try{
						fields_L.clear();
						boost::split(fields_L, fields[i], boost::algorithm::is_any_of("?"));
						if ( fields_L.size() >= query_pub_L_.size() + 1){
							//break;
							if (fields_L.size() > 0 && fields_L[0] == "L"){
								for (int i = 0; i < fields_L.size()-1; ++i){
									sub_fields_L.clear();
									boost::split(sub_fields_L, fields_L[i + 1], boost::algorithm::is_any_of(":"));
									
									roboteq_motor_controller_driver::channel_values msg;
									//msg.header.stamp = current_time;

									for (int j = 0; j < sub_fields_L.size(); j++){
										try{
											msg.value.push_back(boost::lexical_cast<int>(sub_fields_L[j]));
										}
										catch (const std::exception &e){
											ROS_ERROR_STREAM(tag << "Low Garbage data on Serial");
											std::cerr << e.what() << '\n';
										}
									}
									query_pub_L_[i].publish(msg);
								}
							}
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
		else{
			ROS_WARN_STREAM(tag << "Unknown:{" << result.data << "}");
		}
	}
}

void RoboteqDriver::formQuery(std::string param, 
							std::map<std::string,std::string> &queries, 
							std::vector<ros::Publisher> &pubs,
							std::stringstream &ser_str){
	nh_.getParam(param, queries);
	for (std::map<std::string, std::string>::iterator iter = queries.begin(); iter != queries.end(); iter++){
		ROS_INFO_STREAM(tag << param  <<" Publish topic: " << iter->first);
		pubs.push_back(nh_.advertise<roboteq_motor_controller_driver::channel_values>(iter->first, 100));

		std::string cmd = iter->second;
		ser_str << cmd << "_";
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_motor_controller_driver");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	RoboteqDriver driver(nh, nh_priv);

	ros::MultiThreadedSpinner spinner(8);
	spinner.spin();
	ros::waitForShutdown();

	return 0;
}
