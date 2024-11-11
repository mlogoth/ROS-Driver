#include <mutex>
#include <math.h>
#include <thread>
#include <sstream>
#include <cassert>
#include <iostream>
#include <typeinfo>

#include <tf/tf.h>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>

#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>

#include <roboteq_motor_controller_driver/config_srv.h>
#include <roboteq_motor_controller_driver/command_srv.h>
#include <roboteq_motor_controller_driver/channel_values.h>
#include <roboteq_motor_controller_driver/maintenance_srv.h>

template <typename T> float sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

static const std::string tag{"[RoboteQ] "};

class RoboteqDriver
{
public:
	RoboteqDriver(ros::NodeHandle nh, ros::NodeHandle nh_priv);

	~RoboteqDriver()
	{
		if (ser_.isOpen())
		{
			ser_.close();
		}
	}

	void run();

private:	
	void connect();
	void initialize();
	bool read_i2t_parameters();
	void read_roboteq_output();
	void initialize_services();
	void dual_vel_callback(const std_msgs::Int16 &msg);
	void channel_1_vel_callback(const std_msgs::Int16 &msg);
	void channel_2_vel_callback(const std_msgs::Int16 &msg);
	void skid_steering_vel_callback(const geometry_msgs::Twist &msg);
	bool resetstoservice(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response);
	void rpm_mapping(const double &right_speed, const double &left_speed, double &right_speed_cr, double &left_speed_cr);
	void formQuery(std::string, std::map<std::string, std::string> &, std::vector<ros::Publisher> &, std::stringstream &);	
	bool configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response);
	bool commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response);
	bool maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response);

	ros::NodeHandle n;
	ros::NodeHandle nh;
	ros::NodeHandle nh_;
	ros::NodeHandle nh_priv_;

	ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer resetstosrv;
	ros::ServiceServer maintenancesrv;

	ros::Subscriber cmd_vel_sub;
	ros::Subscriber cmd_vel_channel_1_sub;
	ros::Subscriber cmd_vel_channel_2_sub;

	ros::Publisher i2t_pub_;
	ros::Publisher serial_read_pub_;

	std::vector<int> f_list; // a list of frequencies for the queries to be published
	std::vector<int> cum_query_size;
	std::vector<ros::Publisher> query_pub_;

	int32_t baud;
	std::string port;
	serial::Serial ser_;
	
	int rate;
	bool safe_speed;	
	double max_vel_x;
	double max_vel_ang;
	double track_width;
	int max_rpm = 2600;
	float safe_speed_scale;
	double reduction_ratio;
	double wheel_circumference;	

	std::string motor_type;
	std::string motor_1_type;
	std::string motor_2_type;
	std::string channel_mode;

	// I2T parameters
	int motor_amps_index_{-1};
	int runtime_status_flags_index_{-1};
	std::vector<double> amp_limit_;
	std::vector<double> i2t_limit_;
	std::vector<double> time_amp_limit_;
	std::vector<double> nominal_current_;	
	roboteq_motor_controller_driver::channel_values motor_amps_;
	roboteq_motor_controller_driver::channel_values runtime_status_flags_;	
};

RoboteqDriver::RoboteqDriver(ros::NodeHandle nh, ros::NodeHandle nh_priv) : nh_(nh), nh_priv_(nh_priv)
{
	initialize();
}

void RoboteqDriver::initialize()
{
	nh_.getParam("port", port);
	nh_.getParam("baud", baud);

	if (!nh_.getParam("rate", rate))
	{
		rate = 5;
	}

	if (!nh_.getParam("max_rpm", max_rpm))
	{
		max_rpm = 2600;
	}

	if (!nh_.getParam("channel_mode", channel_mode))
	{
		ROS_INFO_STREAM("No channel mode was selected. Assigning driver as dual.");
		channel_mode = "dual";
	}

	if (!nh_.getParam("safe_speed", safe_speed))
	{

		safe_speed = false;
	}

	if (channel_mode == "dual")
	{
		if (!nh_.getParam("motor_type", motor_type))
		{
			ROS_INFO_STREAM("No valid general motor type was selected. Assigning skid_steering and checking for individual motor types.");
			motor_type = "skid_steering";
			if (!nh_.getParam("motor_1_type", motor_1_type))
			{
				ROS_INFO_STREAM("No valid type was found for motor 1. Assigning set_speed");
				motor_1_type = "set_speed";
			}
			if (!nh_.getParam("motor_2_type", motor_2_type))
			{
				ROS_INFO_STREAM("No valid type was found for motor 2. Assigning set_speed");
				motor_2_type = "set_speed";
			}
		}
		else
		{
			motor_1_type = motor_type;
			motor_2_type = motor_type;
		}
	}
	else // single
	{
		if (!nh_.getParam("motor_1_type", motor_type))
		{
			ROS_INFO_STREAM("No valid type was found for motor 1. Assigning set_speed");
			motor_1_type = "set_speed";
		}
		if (!nh_.getParam("motor_2_type", motor_type))
		{
			ROS_INFO_STREAM("No valid type was found for motor 2. Assigning set_speed");
			motor_2_type = "set_speed";
		}
	}

	if (motor_type == "skid_steering")
	{
		nh_.getParam("track_width", track_width);

		if (!nh_.getParam("max_vel_x", max_vel_x))
		{
			max_vel_x = 0.5;
		}
		
		if (!nh_.getParam("max_vel_ang", max_vel_ang))
		{
			max_vel_ang = 0.5;
		}
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
		cmd_vel_channel_1_sub = nh_.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_1_vel_callback, this);
		cmd_vel_channel_2_sub = nh_.subscribe("dual_cmd_vel", 10, &RoboteqDriver::channel_2_vel_callback, this);
		if ((motor_1_type == "set_speed") || (motor_type == "set_speed"))
		{
			ROS_INFO_STREAM("Motor 1 is operating in closed loop mode.");
		}
		else
		{
			ROS_INFO_STREAM("Motor 1 is operating in open loop mode.");
		}
		if ((motor_2_type == "set_speed") || (motor_type == "set_speed"))
		{
			ROS_INFO_STREAM("Motor 2 is operating in closed loop mode.");
		}
		else
		{
			ROS_INFO_STREAM("Motor 2 is operating in open loop mode.");
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

void RoboteqDriver::rpm_mapping(const double &right_speed, const double &left_speed, double &right_speed_cr, double &left_speed_cr)
{
	double max_vel_wheel= (wheel_circumference*max_rpm)/(reduction_ratio * 60);

	double mx = std::max(fabs(right_speed),fabs(left_speed));
	double ln = 1.0;
	if (mx>max_vel_wheel)
		ln = max_vel_wheel/mx;
	// std::cout << "Max RPM: "<<max_rpm << std::endl;
	right_speed_cr = ln*right_speed;
	left_speed_cr = ln*left_speed;
	// std::cout<<"================================="<<std::endl;
	// std::cout<<"max_vel_wheel: "<<max_vel_wheel<<std::endl;
	// std::cout<<"right_speed: "<<right_speed<<std::endl;
	// std::cout<<"left_speed: "<<left_speed<<std::endl;
	// std::cout<<"right_speed_cr: "<<right_speed_cr<<std::endl;
	// std::cout<<"left_speed_cr: "<<left_speed_cr<<std::endl;

}

void RoboteqDriver::skid_steering_vel_callback(const geometry_msgs::Twist &msg)
{

	// max linear speed and angular
	/* 
	max_wheel_speed = wheel_circumference*max_rpm/(reduction_ratio*60)
	max_linear_speed = 0.5 * max_wheel_speed
	max_angular_speed = 0.5 * 2.0*max_wheel_speed/track_width
	*/ 

	double a = 1.0;

	if (!nh_.getParam("safe_speed", safe_speed))
	{

		safe_speed = false;
	}

	if (!nh_.getParam("safe_speed_scale", safe_speed_scale))
	{
		safe_speed_scale = 0.6;
	}

	if (safe_speed){
		ROS_WARN_STREAM("Robot Speed in safe Mode!");
		a = safe_speed_scale;
		if ( a>1.0 && a<0.05){
			a = 0.6;
		}
	}
	// std::cout << "a: "<< a << std::endl;
	double vw = a*msg.angular.z; 
	double vx = a*msg.linear.x;
	// std::cout << "Initial Linear: "<<msg.linear.x<<"| angular: "<<msg.angular.z << std::endl;
	// std::cout << "vx: "<< vx << std::endl;
	// std::cout << "vw: "<< vw << std::endl;
	if (fabs(vx) > max_vel_x)
	{
		vx = sgn(vx)*max_vel_x;
	}

	if (fabs(vw) > max_vel_ang)
	{
		vw = sgn(vw)*max_vel_ang;
	}

	// wheel speed (m/s)
	double right_speed = vx - track_width * vw / 2.0;
	double left_speed = vx + track_width * vw / 2.0;

	// ROS_INFO_STREAM("================================");
	// ROS_INFO_STREAM("right_speed: " << right_speed);
	// ROS_INFO_STREAM("left_speed: " << left_speed);
	double right_speed_cr;
	double left_speed_cr;
	rpm_mapping(right_speed,left_speed,right_speed_cr,left_speed_cr);
	
	// std::stringstream cmd_sub;
	// ROS_INFO_STREAM("================================");
	// ROS_INFO_STREAM("right_speed: " << right_speed);
	// ROS_INFO_STREAM("left_speed: " << left_speed);

	int32_t right_rpm = (right_speed_cr * reduction_ratio * 60.0) / (wheel_circumference);
	int32_t left_rpm = (left_speed_cr * reduction_ratio * 60.0) / (wheel_circumference);

	// ROS_INFO_STREAM(reduction_ratio);
	// ROS_INFO_STREAM("================================");
	// ROS_INFO_STREAM("right_rpm: " << right_rpm);
	// ROS_INFO_STREAM("left_rpm: " << left_rpm);

	std::stringstream right_cmd;
	std::stringstream left_cmd;

	right_cmd << "!S 1 " << (int)(right_rpm) << "\r";
	left_cmd << "!S 2 " << (int)(left_rpm) << "\r";

	// ROS_INFO_STREAM("----------------------------");
	// ROS_INFO_STREAM("Wheel Motors: right_rpm: "<<right_rpm << " - left_rpm: "<<left_rpm);

	ser_.write(right_cmd.str());
	ser_.write(left_cmd.str());
	ser_.flush();
}

void RoboteqDriver::dual_vel_callback(const std_msgs::Int16 &msg)
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

void RoboteqDriver::channel_1_vel_callback(const std_msgs::Int16 &msg)
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

	else
	{
		ROS_ERROR("Channel 1: Not Valid Motor Type");
	}

	// std::cout << "-- Channel 1 Callback: Type: "<<motor_1_type <<  "| CMD: "<< cmd << "|Channel Command: " << channel_1_cmd.str()<< std::endl;

	ser_.write(channel_1_cmd.str());
	ser_.flush();
}

void RoboteqDriver::channel_2_vel_callback(const std_msgs::Int16 &msg)
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
	else
	{
		ROS_ERROR("Channel 2: Not Valid Motor Type");
	}

	// std::cout << "-- Channel 2 Callback: Type: "<<motor_2_type <<  "|CMD: "<< cmd << "|Channel Command: " << channel_2_cmd.str()<< std::endl;

	// ser_.write("!AC 2 2000_");
	// ser_.write("!DC 2 50_");
	ser_.write(channel_2_cmd.str());
	ser_.flush();
}

void RoboteqDriver::connect()
{
	try
	{
		ser_.setPort(port);
		ser_.setBaudrate(baud); // get baud as param
		serial::Timeout to = serial::Timeout::simpleTimeout(10);
		ser_.setTimeout(to);
		ser_.open();
		// Check STO signals
		ser_.write("!STT\r");
	}
	catch (serial::IOException &e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		ROS_INFO_STREAM("Unable to open port");
	}
	if (ser_.isOpen())
	{
		ROS_INFO_STREAM("Serial Port initialized\"");
	}
	else
	{
		ROS_INFO_STREAM("Serial Port is not open");
	}
	run();
}

void RoboteqDriver::run()
{
	initialize_services();

	// Read I2T parameters
	if (!read_i2t_parameters())
	{
		ROS_ERROR("Reading I2T parameters failed");
	}	

	nh_.getParam("frequency_list", f_list);

	std::stringstream ss_gen;
	ss_gen << "^echof 1_"; // Disable echo from driver
	ss_gen << "# c_";	   // Clear Buffer History of previous queries

	for (int i = 0; i < f_list.size(); i++)
	{
		if (f_list[i] > 0)
		{
			ROS_INFO_STREAM(tag << "frequency " << i << " " << f_list[i]);
			ss_gen << "/\"DF" << i << "?\",\"?\"";
			std::map<std::string, std::string> query_map;
			std::stringstream query_name;
			query_name << "query" << i;
			formQuery(query_name.str(), query_map, query_pub_, ss_gen);
			ss_gen << "# " << 1000 / f_list[i] << "_";
		}
		else
		{
			ROS_ERROR_STREAM(tag << "Negative frequency detected " << f_list[i]);
		}
	}

	ser_.write(ss_gen.str()); // Send commands and queries
	ser_.flush();
	ROS_INFO_STREAM(tag << ss_gen.str());
	serial_read_pub_ = nh_.advertise<std_msgs::String>("read_serial", 1000);
	i2t_pub_ = nh_.advertise<roboteq_motor_controller_driver::channel_values>("i2t", 1000);

	double max_freq;
	max_freq = *std::max_element(f_list.begin(), f_list.end());
	ROS_INFO_STREAM(tag << " max frequency " << max_freq);

	std::thread{std::bind(&RoboteqDriver::read_roboteq_output, this)}.detach();
}

void RoboteqDriver::initialize_services()
{
	n = ros::NodeHandle();
	configsrv = n.advertiseService("config_service", &RoboteqDriver::configservice, this);
	commandsrv = n.advertiseService("command_service", &RoboteqDriver::commandservice, this);
	maintenancesrv = n.advertiseService("maintenance_service", &RoboteqDriver::maintenanceservice, this);
	resetstosrv = n.advertiseService("reset_sto",&RoboteqDriver::resetstoservice, this);
}

bool RoboteqDriver::configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response)
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

bool RoboteqDriver::commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response)
{
	std::stringstream str;
	str << "!" << request.userInput << " " << request.channel << " " << request.value << "_";
	ser_.write(str.str());
	ser_.flush();
	response.result = str.str();

	ROS_INFO_STREAM(response.result);
	return true;
}

bool RoboteqDriver::maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response)
{
	std::stringstream str;
	str << "%" << request.userInput << " "
		<< "_";
	ser_.write(str.str());
	ser_.flush();
	response.result = ser_.read(ser_.available());

	ROS_INFO(response.result.c_str());
	return true;
}

bool RoboteqDriver::resetstoservice(std_srvs::Trigger::Request &request, std_srvs::Trigger::Response &response)
{	
	ser_.write("!STT\r");
	ser_.flush();
	response.success=true; //= ser_.read(ser_.available());
	response.message="STO Test message sent to serial";
	//ROS_INFO(response.message.c_str());
	return true;
}

bool RoboteqDriver::read_i2t_parameters()
{
	// Read I2T parameters
	// Flush both input and output to sync
	ser_.flush();
	while (ser_.available())
	{
		ser_.read(ser_.available());
	}
	
	std::stringstream ss_i2t;
	ss_i2t << "^echof 1_"; // Disable echo from driver
	ss_i2t << "# c_";	   // Clear Buffer History of previous queries
	ss_i2t << "~ALIM_~NOMA_~TPAL_"; // Read Amps Limit, Nominal Current and Time for Amps Limit
	ser_.write(ss_i2t.str()); // Send read commands
	ser_.flush();

	ros::Duration(1.0).sleep();

	std::string result;
	std::string delimiter{"\r"};
	size_t max_response_size = 65536;
	
	while (ser_.available())
	{
		result = ser_.readline(max_response_size, delimiter); // Read up to delimiter		
		result = result.substr(0, result.size()-1); // Remove delimiter from end

		std::vector<std::string> query_fields;
		std::vector<std::string> sub_query_fields;

		if (result.rfind("ALIM", 0) == 0) // if result starts with ALIM
		{
			query_fields.clear();
			boost::split(query_fields, result, boost::algorithm::is_any_of("="));
			if (query_fields.size() != 2)
			{
				ROS_ERROR_STREAM(tag << "Serial data is not as expected");
				return false;
			}
			
			sub_query_fields.clear();
			boost::split(sub_query_fields, query_fields[1], boost::algorithm::is_any_of(":"));

			if (sub_query_fields.size() < 1)
			{
				ROS_ERROR_STREAM(tag << "Serial data is not as expected");
				return false;
			}

			for (int k = 0; k < sub_query_fields.size(); k++)
			{
				try
				{
					amp_limit_.push_back(boost::lexical_cast<int>(sub_query_fields[k])/10.0);
				}
				catch (const std::exception &e)
				{
					ROS_ERROR_STREAM(tag << "Garbage data on Serial " << result << "//" << query_fields[1] << "//" << sub_query_fields[k]);
					std::cerr << e.what() << '\n';
					return false;
				}
			}		
		}
		else if (result.rfind("NOMA", 0) == 0)
		{
			query_fields.clear();
			boost::split(query_fields, result, boost::algorithm::is_any_of("="));
			if (query_fields.size() != 2)
			{
				ROS_ERROR_STREAM(tag << "Serial data is not as expected");
				return false;
			}
			
			sub_query_fields.clear();
			boost::split(sub_query_fields, query_fields[1], boost::algorithm::is_any_of(":"));

			if (sub_query_fields.size() < 1)
			{
				ROS_ERROR_STREAM(tag << "Serial data is not as expected");
				return false;
			}

			for (int k = 0; k < sub_query_fields.size(); k++)
			{
				try
				{
					nominal_current_.push_back(boost::lexical_cast<int>(sub_query_fields[k])/10.0);
				}
				catch (const std::exception &e)
				{
					ROS_ERROR_STREAM(tag << "Garbage data on Serial " << result << "//" << query_fields[1] << "//" << sub_query_fields[k]);
					std::cerr << e.what() << '\n';
					return false;
				}
			}
		}
		else if (result.rfind("TPAL", 0) == 0)
		{
			query_fields.clear();
			boost::split(query_fields, result, boost::algorithm::is_any_of("="));
			if (query_fields.size() != 2)
			{
				ROS_ERROR_STREAM(tag << "Serial data is not as expected");
				return false;
			}
			
			sub_query_fields.clear();
			boost::split(sub_query_fields, query_fields[1], boost::algorithm::is_any_of(":"));

			if (sub_query_fields.size() < 1)
			{
				ROS_ERROR_STREAM(tag << "Serial data is not as expected");
				return false;
			}

			for (int k = 0; k < sub_query_fields.size(); k++)
			{
				try
				{
					time_amp_limit_.push_back(boost::lexical_cast<int>(sub_query_fields[k])*1.0);
				}
				catch (const std::exception &e)
				{
					ROS_ERROR_STREAM(tag << "Garbage data on Serial " << result << "//" << query_fields[1] << "//" << sub_query_fields[k]);
					std::cerr << e.what() << '\n';
					return false;
				}
			}
		}
	}		

	if (amp_limit_.size() == nominal_current_.size() && amp_limit_.size() == time_amp_limit_.size())
	{
		for (int i = 0; i < amp_limit_.size(); i++)
		{
			i2t_limit_.push_back((amp_limit_[i]*amp_limit_[i] - nominal_current_[i]*nominal_current_[i])*time_amp_limit_[i]);
			ROS_INFO_STREAM(tag << "Channel " << i << ", Amp Limit: " << amp_limit_[i] << ", Nominal Current: " << nominal_current_[i] << ", Time Amp Limit: " << time_amp_limit_[i] << ", I2T Limit: " << i2t_limit_[i]);
		}
		return true;
	}
	else
	{
		ROS_ERROR_STREAM(tag << "Number of channels is not consistent among parameters");
		return false;
	}	
}

void RoboteqDriver::formQuery(std::string param, std::map<std::string, std::string> &queries, std::vector<ros::Publisher> &pubs, std::stringstream &ser_str)
{
	nh_.getParam(param, queries);
	int count = 0;
	for (std::map<std::string, std::string>::iterator iter = queries.begin(); iter != queries.end(); iter++)
	{
		ROS_INFO_STREAM(tag << param << " Publish topic: " << iter->first);
		pubs.push_back(nh_.advertise<roboteq_motor_controller_driver::channel_values>(iter->first, 100));

		std::string cmd = iter->second;
		ser_str << cmd << "_";
		
		// Find motor amps and runtime status flags
		if (iter->second == "?A")
		{
			motor_amps_index_ = count + (cum_query_size.empty() ? 0 : cum_query_size.back());
		}
		else if (iter->second == "?FM")
		{
			runtime_status_flags_index_ = count + (cum_query_size.empty() ? 0 : cum_query_size.back());
		}

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

void RoboteqDriver::read_roboteq_output()
{
	int frequency_index;
	std::string message;
	std::string data_copy;	
	std_msgs::String result;
	std_msgs::Header header;
	std::string delimiter{"\r"};
	size_t max_response_size = 65536;
	std::vector<std::string> query_fields;
	std::vector<std::string> sub_query_fields;
	roboteq_motor_controller_driver::channel_values msg;
	roboteq_motor_controller_driver::channel_values i2t_flag;

	// Flush both input and output to sync
	ser_.flush();
	while (ser_.available())
	{
		ser_.read(ser_.available());
	}

	while (ros::ok() && ser_.isOpen())
	{
		message = ser_.readline(max_response_size, delimiter); // Read up to delimiter
		if (message.size() == 0 || (message[0] != 'D' && message[1] != 'F'))
		{	
			continue;
		}
		
		// Save time
		header.stamp = ros::Time::now();

		// Remove delimiter from end
		message = message.substr(0, message.size()-1);

		// Decode
		frequency_index = boost::lexical_cast<int>(message[2]);

		try
		{
			boost::split(query_fields, message, boost::algorithm::is_any_of("?"));
			for (int j = 1; j < query_fields.size(); j++)
			{				
				msg.value.clear();
				msg.header = header;

				sub_query_fields.clear();				

				boost::split(sub_query_fields, query_fields[j], boost::algorithm::is_any_of(":"));
				
				for (int k = 0; k < sub_query_fields.size(); k++)
				{
					try
					{
						msg.value.push_back(boost::lexical_cast<int>(sub_query_fields[k]));
					}
					catch (const std::exception &e)
					{

						ROS_ERROR_STREAM(tag << "Garbage data on Serial " << message << "//" << query_fields[j] << "//" << sub_query_fields[k]);
						std::cerr << e.what() << '\n';
						break;
					}
				}
				query_pub_[cum_query_size[frequency_index] + j - 1].publish(msg);

				// Save motor amps values and runtime status flags for I2T check
				if (cum_query_size[frequency_index] + j - 1 == motor_amps_index_)
				{
					motor_amps_ = msg;
				}
				else if (cum_query_size[frequency_index] + j - 1 == runtime_status_flags_index_)
				{
					runtime_status_flags_ = msg;
				}
			}
		}
		catch (const std::exception &e)
		{
			std::cerr << e.what() << '\n';
			ROS_ERROR_STREAM(tag << "Finding query output in :" << message);
		}

		// I2T check
		i2t_flag.value.clear();
		i2t_flag.header = header;
		if (motor_amps_index_ != -1 && runtime_status_flags_index_ != -1 && runtime_status_flags_.value.size() == motor_amps_.value.size())
		{
			for(int i=0; i < runtime_status_flags_.value.size(); i++)
			{
				i2t_flag.value.push_back(0);
				// Check if Amp Limit is present
				if (runtime_status_flags_.value[i] % 2 == 1)
				{
					if (std::abs(motor_amps_.value[i])/10.0 > 1.1 * nominal_current_[i])
					{
						ROS_WARN_STREAM(tag << "AmpLim protection in Channel: " << i);
					}
					else
					{
						ROS_WARN_STREAM(tag << "I2T protection in Channel: " << i);
						i2t_flag.value.back()= 1;
					}
				}
			}
			i2t_pub_.publish(i2t_flag);
		}
		else
		{
			ROS_ERROR_STREAM(tag << "Cannot perform I2T check");
		}

		// Clear query fields
		query_fields.clear();

		// Publish raw data
		result.data = message;
		serial_read_pub_.publish(result); 
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "roboteq_motor_controller_driver");

	ros::NodeHandle nh;
	ros::NodeHandle nh_priv("~");
	RoboteqDriver driver(nh, nh_priv);

	ros::spin();
	ros::waitForShutdown();

	return 0;
}