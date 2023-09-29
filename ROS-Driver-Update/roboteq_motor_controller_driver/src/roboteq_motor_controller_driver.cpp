#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver.h>

// #include <boost/lexical_cast.hpp>
// #include <boost/algorithm/string/split.hpp>
// #include <boost/algorithm/string/trim.hpp>
// #include <boost/algorithm/string/replace.hpp>
// #include <boost/algorithm/string/classification.hpp>

// #include <ros/ros.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
// #include <std_msgs/Bool.h>
// #include <std_msgs/Int16.h>
// #include <iostream>
// #include <sstream>
// #include <typeinfo>
// #include <cassert>
// #include <mutex>
// #include <math.h>
// #include <geometry_msgs/Twist.h>
// #include <nav_msgs/Odometry.h>
// #include <tf/tf.h>
// #include <roboteq_motor_controller_driver/querylist.h>
// #include <roboteq_motor_controller_driver/channel_values.h>
// #include <roboteq_motor_controller_driver/config_srv.h>
// #include <roboteq_motor_controller_driver/command_srv.h>
// #include <roboteq_motor_controller_driver/maintenance_srv.h>

using std::placeholders::_1;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("Roboteq");

namespace roboteq
{

RoboteqDriver::RoboteqDriver() : Node("roboteq_motor_controller_driver")
{
	// Get Parameters
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
	
	if (params_.motor_1_type == "set_speed")
	{
		RCLCPP_INFO(LOGGER, "Motor 1 is operating in closed loop mode.");
	}
	else
	{
		RCLCPP_INFO(LOGGER, "Motor 1 is operating in open loop mode.");
	}

	if (params_.motor_2_type == "set_speed")
	{
		RCLCPP_INFO(LOGGER, "Motor 2 is operating in closed loop mode.");
	}
	else
	{
		RCLCPP_INFO(LOGGER, "Motor 2 is operating in open loop mode.");
	}

	if (params_.channel_mode == "dual_differential")
	{
		RCLCPP_INFO(LOGGER, "Driver controls two motors moving a skid steering vehicle.");
		cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::skid_steering_vel_callback, this, _1));
	}
	else if (params_.channel_mode == "dual")
	{
		RCLCPP_INFO(LOGGER, "Driver controls two motors with a single value.");
		cmd_vel_channel_1_sub = this->create_subscription<std_msgs::msg::Int16>("dual_cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_1_vel_callback, this, _1));
		cmd_vel_channel_2_sub = this->create_subscription<std_msgs::msg::Int16>("dual_cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_2_vel_callback, this, _1));
	}
	else if (params_.channel_mode == "single")
	{
		RCLCPP_INFO(LOGGER, "Driver controls two motors seperately.");
		if (params_.motor_1_type == "set_speed")
		{
			cmd_vel_channel_1_sub = this->create_subscription<std_msgs::msg::Int16>("chan_1_set_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_1_vel_callback, this, _1));
		}
		else
		{
			cmd_vel_channel_1_sub = this->create_subscription<std_msgs::msg::Int16>("chan_1_go_to_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_1_vel_callback, this, _1));
		}
		if (params_.motor_2_type == "set_speed")
		{
			cmd_vel_channel_2_sub = this->create_subscription<std_msgs::msg::Int16>("chan_2_set_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_2_vel_callback, this, _1));
		}
		else
		{
			cmd_vel_channel_2_sub = this->create_subscription<std_msgs::msg::Int16>("chan_2_go_to_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_2_vel_callback, this, _1));
		}
	}

	// Print parameters
	RCLCPP_INFO_STREAM(LOGGER, "port: " << params_.serial.port);
	RCLCPP_INFO_STREAM(LOGGER, "baud: " << params_.serial.baud);
	RCLCPP_INFO_STREAM(LOGGER, "channel_mode: " << params_.channel_mode);
	RCLCPP_INFO_STREAM(LOGGER, "motor_1_type: " << params_.motor_1_type);
	RCLCPP_INFO_STREAM(LOGGER, "motor_2_type: " << params_.motor_2_type);
	RCLCPP_INFO_STREAM(LOGGER, "safe_speed: " << params_.differential.safe_speed);
	RCLCPP_INFO_STREAM(LOGGER, "max_vel_x: " << params_.differential.max_vel_x);
	RCLCPP_INFO_STREAM(LOGGER, "max_vel_ang: " << params_.differential.max_vel_ang);
	RCLCPP_INFO_STREAM(LOGGER, "track_width: " << params_.mechanical.track_width);
	RCLCPP_INFO_STREAM(LOGGER, "reduction_ratio: " << params_.mechanical.reduction_ratio);
	RCLCPP_INFO_STREAM(LOGGER, "wheel_circumference: " << params_.mechanical.wheel_circumference);

	connect();
	initialize_services();
	run();
}

RoboteqDriver::~RoboteqDriver()
{
	if (ser_.isOpen())
	{
		ser_.close();
	}
}

void RoboteqDriver::connect()
{
	try
	{
		ser_.setPort(params_.serial.port);
		ser_.setBaudrate(params_.serial.baud);
		serial::Timeout to = serial::Timeout::simpleTimeout(10);
		ser_.setTimeout(to);
		ser_.open();
	}
	catch (serial::IOException &e)
	{

		RCLCPP_ERROR(LOGGER, "Unable to open port");
	}
	if (ser_.isOpen())
	{

		RCLCPP_INFO(LOGGER, "Serial Port initialized");
	}
	else
	{
		RCLCPP_INFO(LOGGER, "Serial Port is not open");
	}
}

void RoboteqDriver::initialize_services()
{
	config_srv = this->create_service<roboteq_motor_controller_msgs::srv::Config>("config_service", &RoboteqDriver::configservice);
	command_srv = this->create_service<roboteq_motor_controller_msgs::srv::Command>("command_service", &RoboteqDriver::commandservice);
	maintenance_srv = this->create_service<roboteq_motor_controller_msgs::srv::Maintenance>("maintenance_service", &RoboteqDriver::maintenanceservice);
}

bool RoboteqDriver::configservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Config::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Config::Response> response)
{
	std::stringstream str;
	str << "^" << request->user_input << " " << request->channel << " " << request->value << "_ " << "%\clsav321654987";
	ser_.write(str.str());
	ser_.flush();
	response->result = str.str();

	RCLCPP_INFO_STREAM(LOGGER, response->result);
	return true;
}

bool RoboteqDriver::commandservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Command::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Command::Response> response)
{
	std::stringstream str;
	str << "!" << request->user_input << " " << request->channel << " " << request->value << "_";
	ser_.write(str.str());
	ser_.flush();
	response->result = str.str();

	RCLCPP_INFO_STREAM(LOGGER, response->result);
	return true;
}

bool RoboteqDriver::maintenanceservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Maintenance::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Maintenance::Response> response)
{
	std::stringstream str;
	str << "%" << request->user_input << " " << "_";
	ser_.write(str.str());
	ser_.flush();
	response->result = ser_.read(ser_.available());

	RCLCPP_INFO_STREAM(LOGGER, response->result);
	return true;
}

void RoboteqDriver::run()
{
	std::stringstream ss_gen;
	ss_gen << "^echof 1_"; // Disable echo from driver
	ss_gen << "# c_";	   // Clear Buffer History of previous queries

	auto f_list = params_.frequency_list;

	for (int i = 0; i < f_list.size(); i++)
	{
		RCLCPP_INFO_STREAM(LOGGER, "Frequency " << i << " " << f_list[i]);
		ss_gen << "/\"DF" << i << "?\",\"?\"";
		std::map<std::string, std::string> query_map;
		std::stringstream query_name;
		query_name << "query" << i;
		formQuery(query_name.str(), query_map, query_pub_, ss_gen);
		ss_gen << "# " << 1000 / f_list[i] << "_";
	}

	ser_.write(ss_gen.str()); // Send commands and queries
	ser_.flush();
	RCLCPP_INFO(LOGGER, ss_gen.str());
	serial_read_pub_ = this->create_publisher<std_msgs::msg::String>("read_serial", rclcpp::SystemDefaultsQoS());

	double max_freq;
	max_freq = *std::max_element(f_list.begin(), f_list.end());
	RCLCPP_INFO_STREAM(LOGGER, "Max frequency " << max_freq);
	previous_time = this->get_clock()->now();
	// NOT FINISHED
	rclcpp::create_timer(this->get_clock(), rclcpp::Duration::from_seconds(1.0/max_freq), &RoboteqDriver::queryCallback);
	// timer_pub_ = nh_.createTimer(ros::Duration(double(1.0 / max_freq)), &RoboteqDriver::queryCallback, this);
}

// 	void RoboteqDriver::rpm_mapping(const double &right_speed, const double &left_speed, double &right_speed_cr, double &left_speed_cr)
// 	{
// 		double max_vel_wheel = (wheel_circumference*max_rpm)/(reduction_ratio * 60);

// 		double mx = std::max(fabs(right_speed),fabs(left_speed));
// 		double ln = 1.0;
// 		if (mx > max_vel_wheel)
// 		{
// 			ln = max_vel_wheel/mx;
// 		}	

// 		right_speed_cr = ln*right_speed;
// 		left_speed_cr = ln*left_speed;

// 		// std::cout<<"================================="<<std::endl;
// 		// std::cout<<"max_vel_wheel: "<<max_vel_wheel<<std::endl;
// 		// std::cout<<"right_speed: "<<right_speed<<std::endl;
// 		// std::cout<<"left_speed: "<<left_speed<<std::endl;
// 		// std::cout<<"right_speed_cr: "<<right_speed_cr<<std::endl;
// 		// std::cout<<"left_speed_cr: "<<left_speed_cr<<std::endl;
// 	}

// 	void RoboteqDriver::skid_steering_vel_callback(const geometry_msgs::Twist &msg)
// 	{

// 		// max linear speed and angular
// 		/* 
// 		max_wheel_speed = wheel_circumference*max_rpm/(reduction_ratio*60)
// 		max_linear_speed = 0.5 * max_wheel_speed
// 		max_angular_speed = 0.5 * 2.0*max_wheel_speed/track_width
// 		*/ 

// 		double a = 1.0;

// 		if (!nh_.getParam("safe_speed", safe_speed))
// 		{

// 			safe_speed = false;
// 		}
// 		if (safe_speed){
// 			ROS_WARN_STREAM("Robot Speed in safe Mode!");
// 			a = 0.6;
// 		}
// 		// std::cout << "a: "<< a << std::endl;
// 		double vw = a*msg.angular.z; 
// 		double vx = a*msg.linear.x;
// 		// std::cout << "Initial Linear: "<<msg.linear.x<<"| angular: "<<msg.angular.z << std::endl;
// 		// std::cout << "vx: "<< vx << std::endl;
// 		// std::cout << "vw: "<< vw << std::endl;
// 		if (fabs(vx) > max_vel_x)
// 		{
// 			vx = sgn(vx)*max_vel_x;
// 		}

// 		if (fabs(vw) > max_vel_ang)
// 		{
// 			vw = sgn(vw)*max_vel_ang;
// 		}

// 		// wheel speed (m/s)
// 		double right_speed = vx - track_width * vw / 2.0;
// 		double left_speed = vx + track_width * vw / 2.0;

// 		// ROS_INFO_STREAM("================================");
// 		// ROS_INFO_STREAM("right_speed: " << right_speed);
// 		// ROS_INFO_STREAM("left_speed: " << left_speed);
// 		double right_speed_cr;
// 		double left_speed_cr;
// 		rpm_mapping(right_speed,left_speed,right_speed_cr,left_speed_cr);
		
// 		// std::stringstream cmd_sub;
// 		// ROS_INFO_STREAM("================================");
// 		// ROS_INFO_STREAM("right_speed: " << right_speed);
// 		// ROS_INFO_STREAM("left_speed: " << left_speed);

// 		int32_t right_rpm = (right_speed_cr * reduction_ratio * 60.0) / (wheel_circumference);
// 		int32_t left_rpm = (left_speed_cr * reduction_ratio * 60.0) / (wheel_circumference);

// 		// ROS_INFO_STREAM(reduction_ratio);
// 		// ROS_INFO_STREAM("================================");
// 		// ROS_INFO_STREAM("right_rpm: " << right_rpm);
// 		// ROS_INFO_STREAM("left_rpm: " << left_rpm);

// 		std::stringstream right_cmd;
// 		std::stringstream left_cmd;

// 		right_cmd << "!S 1 " << (int)(right_rpm) << "\r";
// 		left_cmd << "!S 2 " << (int)(left_rpm) << "\r";

// 		// ROS_INFO_STREAM("----------------------------");
// 		// ROS_INFO_STREAM("Wheel Motors: right_rpm: "<<right_rpm << " - left_rpm: "<<left_rpm);

// 		ser_.write(right_cmd.str());
// 		ser_.write(left_cmd.str());
// 		ser_.flush();
// 	}

// 	void RoboteqDriver::dual_vel_callback(const std_msgs::msg::Int16 &msg)
// 	{
// 		// wheel speed (m/s)
// 		int cmd = msg.data;

// 		// ROS_INFO_STREAM("================================");
// 		// ROS_INFO_STREAM("Motor Command:" << cmd);

// 		std::stringstream right_cmd;
// 		std::stringstream left_cmd;

// 		if (motor_type == "go_to_speed")
// 		{
// 			right_cmd << "!G 2 " << cmd << "\r";
// 			left_cmd << "!G 1 " << cmd << "\r";
// 		}
// 		else if (motor_type == "set_speed")
// 		{
// 			right_cmd << "!S 2 " << cmd << "\r";
// 			left_cmd << "!S 1 " << cmd << "\r";
// 		}
// 		ser_.write(right_cmd.str());
// 		ser_.write(left_cmd.str());
// 		ser_.flush();
// 	}

// 	void RoboteqDriver::channel_1_vel_callback(const std_msgs::msg::Int16 &msg)
// 	{
// 		// wheel speed (m/s)
// 		int cmd = msg.data;

// 		std::stringstream channel_1_cmd;

// 		if (motor_1_type == "go_to_speed")
// 		{
// 			channel_1_cmd << "!G 1 " << cmd << "\r";
// 		}
// 		else if (motor_1_type == "set_speed")
// 		{
// 			channel_1_cmd << "!S 1 " << cmd << "\r";
// 		}

// 		else
// 		{
// 			ROS_ERROR("Channel 1: Not Valid Motor Type");
// 		}

// 		// std::cout << "-- Channel 1 Callback: Type: "<<motor_1_type <<  "| CMD: "<< cmd << "|Channel Command: " << channel_1_cmd.str()<< std::endl;

// 		ser_.write(channel_1_cmd.str());
// 		ser_.flush();
// 	}

// 	void RoboteqDriver::channel_2_vel_callback(const std_msgs::msg::Int16 &msg)
// 	{
// 		// wheel speed (m/s)
// 		int cmd = msg.data;

// 		std::stringstream channel_2_cmd;

// 		if (motor_2_type == "go_to_speed")
// 		{
// 			channel_2_cmd << "!G 2 " << cmd << "\r";
// 		}
// 		else if (motor_2_type == "set_speed")
// 		{
// 			channel_2_cmd << "!S 2 " << cmd << "\r";
// 		}
// 		else
// 		{
// 			ROS_ERROR("Channel 2: Not Valid Motor Type");
// 		}

// 		// std::cout << "-- Channel 2 Callback: Type: "<<motor_2_type <<  "|CMD: "<< cmd << "|Channel Command: " << channel_2_cmd.str()<< std::endl;

// 		// ser_.write("!AC 2 2000_");
// 		// ser_.write("!DC 2 50_");
// 		ser_.write(channel_2_cmd.str());
// 		ser_.flush();
// 	}





	

// void RoboteqDriver::queryCallback(const ros::TimerEvent &event)
// {
// 	std_msgs::String result;

// 	// Mutex is probably not needed in single threaded spinning
// 	std::lock_guard<std::mutex> lock(locker); // Lock mutex until end of scope
// 	// std::cout << "---- Loop:  " <<  ros::Time::now().toSec()-previous_time.toSec()<< std::endl;
// 	// std::cout<<"Last Expected: "<<event.last_expected.toSec()- ros::Time::now().toSec()<<std::endl;
// 	// std::cout<<"Last Real: "<<event.last_real.toSec()- ros::Time::now().toSec()<<std::endl;
// 	// std::cout<<"current_expected: "<<event.current_expected.toSec()- ros::Time::now().toSec()<<std::endl;
// 	// std::cout<<"Current Real: "<<event.current_real.toSec()- ros::Time::now().toSec()<<std::endl;

// 	if (ser_.available())
// 	{
// 		result.data = ser_.read(ser_.available()); // Read all available bytes

// 		serial_read_pub_.publish(result); // Publish raw data

// 		std::vector<std::string> fields;

// 		boost::split(fields, result.data, boost::algorithm::is_any_of("\r")); // Split by termination character "\r"

// 		if (fields.size() < 2)
// 		{
// 			ROS_ERROR_STREAM(tag << "Empty data:{" << result.data << "}");
// 			return;
// 		}

// 		std::vector<std::string> query_fields;
// 		std::vector<std::string> sub_query_fields;
// 		int frequency_index;

// 		for (int i = 0; i < fields.size() - 1; i++)
// 		{
// 			if (fields[i].rfind("DF", 0) == 0) // if field starts with "DF"
// 			{
// 				frequency_index = boost::lexical_cast<int>(fields[i][2]);
// 				try
// 				{
// 					query_fields.clear();
// 					boost::split(query_fields, fields[i], boost::algorithm::is_any_of("?"));
// 					for (int j = 1; j < query_fields.size(); j++)
// 					{
// 						sub_query_fields.clear();
// 						boost::split(sub_query_fields, query_fields[j], boost::algorithm::is_any_of(":"));

// 						roboteq_motor_controller_driver::channel_values msg;
// 						for (int k = 0; k < sub_query_fields.size(); k++)
// 						{
// 							try
// 							{
// 								msg.value.push_back(boost::lexical_cast<int>(sub_query_fields[k]));
// 							}
// 							catch (const std::exception &e)
// 							{

// 								ROS_ERROR_STREAM(tag << "Garbage data on Serial " << fields[i] << "//" << query_fields[j] << "//" << sub_query_fields[k]);
// 								std::cerr << e.what() << '\n';
// 								break;
// 							}
// 						}
// 						query_pub_[cum_query_size[frequency_index] + j - 1].publish(msg);
// 					}
// 				}
// 				catch (const std::exception &e)
// 				{
// 					std::cerr << e.what() << '\n';
// 					ROS_ERROR_STREAM(tag << "Finding query output in :" << fields[i]);
// 					continue;
// 				}
// 			}
// 		}
// 	}
// 	previous_time = ros::Time::now();
// }

// void RoboteqDriver::formQuery(std::string param,
// 							  std::map<std::string, std::string> &queries,
// 							  std::vector<ros::Publisher> &pubs,
// 							  std::stringstream &ser_str)
// {
// 	nh_.getParam(param, queries);
// 	int count = 0;
// 	for (std::map<std::string, std::string>::iterator iter = queries.begin(); iter != queries.end(); iter++)
// 	{
// 		ROS_INFO_STREAM(tag << param << " Publish topic: " << iter->first);
// 		pubs.push_back(nh_.advertise<roboteq_motor_controller_driver::channel_values>(iter->first, 100));

// 		std::string cmd = iter->second;
// 		ser_str << cmd << "_";
// 		count++;
// 	}
// 	if (!cum_query_size.empty())
// 	{
// 		cum_query_size.push_back(count + cum_query_size.back());
// 	}
// 	else
// 	{
// 		cum_query_size.push_back(0);
// 		cum_query_size.push_back(count);
// 	}
// }

} // roboteq namespace

int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
	rclcpp::executors::MultiThreadedExecutor executor;
	roboteq::RoboteqDriver driver_node{};
	executor.add_node(driver_node.get_node_base_interface());
	executor.spin();
	rclcpp::shutdown();
  	return 0;
}