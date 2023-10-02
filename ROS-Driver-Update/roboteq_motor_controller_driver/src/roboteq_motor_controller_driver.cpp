#include <roboteq_motor_controller_driver/roboteq_motor_controller_driver.h>
#include <chrono>
#include <boost/lexical_cast.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/classification.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
static const rclcpp::Logger LOGGER = rclcpp::get_logger("Roboteq");

namespace roboteq
{

RoboteqDriver::RoboteqDriver() : Node("roboteq_motor_controller_driver")
{
	// Get Parameters
    param_listener_ = std::make_shared<ParamListener>(this->get_node_parameters_interface());
    params_ = param_listener_->get_params();
	
	if (params_.motor.motor_1_type == "set_speed")
	{
		RCLCPP_INFO(LOGGER, "Motor 1 is operating in closed loop mode.");
	}
	else
	{
		RCLCPP_INFO(LOGGER, "Motor 1 is operating in open loop mode.");
	}

	if (params_.motor.motor_2_type == "set_speed")
	{
		RCLCPP_INFO(LOGGER, "Motor 2 is operating in closed loop mode.");
	}
	else
	{
		RCLCPP_INFO(LOGGER, "Motor 2 is operating in open loop mode.");
	}

	if (params_.motor.channel_mode == "dual_differential")
	{
		RCLCPP_INFO(LOGGER, "Driver controls two motors moving a skid steering vehicle.");
		cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::skid_steering_vel_callback, this, _1));
	}
	else if (params_.motor.channel_mode == "dual")
	{
		RCLCPP_INFO(LOGGER, "Driver controls two motors with a single value.");
		cmd_vel_channel_1_sub = this->create_subscription<std_msgs::msg::Int16>("dual_cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_1_vel_callback, this, _1));
		cmd_vel_channel_2_sub = this->create_subscription<std_msgs::msg::Int16>("dual_cmd_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_2_vel_callback, this, _1));
	}
	else if (params_.motor.channel_mode == "single")
	{
		RCLCPP_INFO(LOGGER, "Driver controls two motors seperately.");
		if (params_.motor.motor_1_type == "set_speed")
		{
			cmd_vel_channel_1_sub = this->create_subscription<std_msgs::msg::Int16>("chan_1_set_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_1_vel_callback, this, _1));
		}
		else
		{
			cmd_vel_channel_1_sub = this->create_subscription<std_msgs::msg::Int16>("chan_1_go_to_vel", rclcpp::SystemDefaultsQoS(), std::bind(&RoboteqDriver::channel_1_vel_callback, this, _1));
		}
		if (params_.motor.motor_2_type == "set_speed")
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
	RCLCPP_INFO_STREAM(LOGGER, "channel_mode: " << params_.motor.channel_mode);
	RCLCPP_INFO_STREAM(LOGGER, "motor_1_type: " << params_.motor.motor_1_type);
	RCLCPP_INFO_STREAM(LOGGER, "motor_2_type: " << params_.motor.motor_2_type);
	RCLCPP_INFO_STREAM(LOGGER, "max_rpm: " << params_.motor.max_rpm);
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
	config_srv = this->create_service<roboteq_motor_controller_msgs::srv::Config>("config_service", std::bind(&RoboteqDriver::configservice, this, _1, _2));
	command_srv = this->create_service<roboteq_motor_controller_msgs::srv::Command>("command_service", std::bind(&RoboteqDriver::commandservice, this, _1, _2));
	maintenance_srv = this->create_service<roboteq_motor_controller_msgs::srv::Maintenance>("maintenance_service", std::bind(&RoboteqDriver::maintenanceservice, this, _1, _2));
}

bool RoboteqDriver::configservice(const std::shared_ptr<roboteq_motor_controller_msgs::srv::Config::Request> request, std::shared_ptr<roboteq_motor_controller_msgs::srv::Config::Response> response)
{
	std::stringstream str;
	str << "^" << request->user_input << " " << request->channel << " " << request->value << "_ " << "%clsav321654987";
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

void RoboteqDriver::rpm_mapping(const double &right_speed, const double &left_speed, double &right_speed_cr, double &left_speed_cr)
{
	double max_vel_wheel = (params_.mechanical.wheel_circumference*params_.motor.max_rpm)/(params_.mechanical.reduction_ratio * 60);

	double mx = std::max(fabs(right_speed),fabs(left_speed));
	double ln = 1.0;
	if (mx > max_vel_wheel)
	{
		ln = max_vel_wheel/mx;
	}	

	right_speed_cr = ln*right_speed;
	left_speed_cr = ln*left_speed;
}

void RoboteqDriver::skid_steering_vel_callback(const geometry_msgs::msg::Twist &msg)
{
	double a = 1.0;

	if (params_.differential.safe_speed){
		RCLCPP_WARN(LOGGER, "Robot Speed in safe Mode!");
		a = 0.6;
	}
	
	double vw = a*msg.angular.z; 
	double vx = a*msg.linear.x;
	
	if (fabs(vx) > params_.differential.max_vel_x)
	{
		vx = sgn(vx)*params_.differential.max_vel_x;
	}

	if (fabs(vw) > params_.differential.max_vel_ang)
	{
		vw = sgn(vw)*params_.differential.max_vel_ang;
	}

	// wheel speed (m/s)
	double right_speed = vx - params_.mechanical.track_width * vw / 2.0;
	double left_speed = vx + params_.mechanical.track_width * vw / 2.0;

	double right_speed_cr;
	double left_speed_cr;
	rpm_mapping(right_speed,left_speed,right_speed_cr,left_speed_cr);

	int32_t right_rpm = (right_speed_cr * params_.mechanical.reduction_ratio * 60.0) / (params_.mechanical.wheel_circumference);
	int32_t left_rpm = (left_speed_cr * params_.mechanical.reduction_ratio * 60.0) / (params_.mechanical.wheel_circumference);

	std::stringstream right_cmd;
	std::stringstream left_cmd;

	right_cmd << "!S 1 " << (int)(right_rpm) << "\r";
	left_cmd << "!S 2 " << (int)(left_rpm) << "\r";

	ser_.write(right_cmd.str());
	ser_.write(left_cmd.str());
	ser_.flush();
}

void RoboteqDriver::channel_1_vel_callback(const std_msgs::msg::Int16 &msg)
{
	// wheel speed (m/s)
	int cmd = msg.data;

	std::stringstream channel_1_cmd;

	if (params_.motor.motor_1_type == "go_to_speed")
	{
		channel_1_cmd << "!G 1 " << cmd << "\r";
	}
	else if (params_.motor.motor_1_type == "set_speed")
	{
		channel_1_cmd << "!S 1 " << cmd << "\r";
	}

	ser_.write(channel_1_cmd.str());
	ser_.flush();
}

void RoboteqDriver::channel_2_vel_callback(const std_msgs::msg::Int16 &msg)
{
	// wheel speed (m/s)
	int cmd = msg.data;

	std::stringstream channel_2_cmd;

	if (params_.motor.motor_2_type == "go_to_speed")
	{
		channel_2_cmd << "!G 2 " << cmd << "\r";
	}
	else if (params_.motor.motor_2_type == "set_speed")
	{
		channel_2_cmd << "!S 2 " << cmd << "\r";
	}
	
	ser_.write(channel_2_cmd.str());
	ser_.flush();
}

void RoboteqDriver::queryCallback()
{
	std_msgs::msg::String result;

	if (ser_.available())
	{
		result.data = ser_.read(ser_.available()); // Read all available bytes

		serial_read_pub_->publish(result); // Publish raw data

		std::vector<std::string> fields;

		boost::split(fields, result.data, boost::algorithm::is_any_of("\r")); // Split by termination character "\r"

		if (fields.size() < 2)
		{
			RCLCPP_ERROR_STREAM(LOGGER, "Empty data:{" << result.data << "}");
			return;
		}

		std::vector<std::string> query_fields;
		std::vector<std::string> sub_query_fields;
		int frequency_index;

		for (int i = 0; i < static_cast<int>(fields.size()) - 1; i++)
		{
			if (fields[i].rfind("DF", 0) == 0) // if field starts with "DF"
			{
				frequency_index = boost::lexical_cast<int>(fields[i][2]);
				try
				{
					query_fields.clear();
					boost::split(query_fields, fields[i], boost::algorithm::is_any_of("?"));
					for (int j = 1; j < static_cast<int>(query_fields.size()); j++)
					{
						sub_query_fields.clear();
						boost::split(sub_query_fields, query_fields[j], boost::algorithm::is_any_of(":"));

						roboteq_motor_controller_msgs::msg::ChannelValues msg;
						for (int k = 0; k < static_cast<int>(sub_query_fields.size()); k++)
						{
							try
							{
								msg.value.push_back(boost::lexical_cast<int>(sub_query_fields[k]));
							}
							catch (const std::exception &e)
							{

								RCLCPP_ERROR_STREAM(LOGGER, "Garbage data on Serial " << fields[i] << "//" << query_fields[j] << "//" << sub_query_fields[k]);
								std::cerr << e.what() << '\n';
								break;
							}
						}
						query_pubs_[cum_query_size[frequency_index] + j - 1]->publish(msg);
					}
				}
				catch (const std::exception &e)
				{
					std::cerr << e.what() << '\n';
					RCLCPP_ERROR_STREAM(LOGGER, "Finding query output in :" << fields[i]);
					continue;
				}
			}
		}
	}
}

void RoboteqDriver::formQuery(const std::string query_name, std::stringstream &ser_str)
{
	if(params_.queries.queries_list_map.at(query_name).arguments.size() != params_.queries.queries_list_map.at(query_name).commands.size())
	{
		RCLCPP_ERROR_STREAM(LOGGER, query_name << ": Different size of arguements and commands lists");
		return;
	}

	for (int i = 0; i < static_cast<int>(params_.queries.queries_list_map.at(query_name).arguments.size()); i++)
	{
		RCLCPP_INFO_STREAM(LOGGER, query_name << ": Publish topic: " << params_.queries.queries_list_map.at(query_name).arguments[i]);
		query_pubs_.push_back(this->create_publisher<roboteq_motor_controller_msgs::msg::ChannelValues>(params_.queries.queries_list_map.at(query_name).arguments[i], rclcpp::SystemDefaultsQoS()));

		ser_str << params_.queries.queries_list_map.at(query_name).commands[i] << "_";
	}
	
	cum_query_size.push_back(cum_query_size.back() + params_.queries.queries_list_map.at(query_name).arguments.size());
}

void RoboteqDriver::run()
{
	std::stringstream ss_gen;
	ss_gen << "^echof 1_"; // Disable echo from driver
	ss_gen << "# c_";	   // Clear Buffer History of previous queries

	int max_frequency{0};

	for (int i = 0; i < static_cast<int>(params_.queries_list.size()); i++)
	{
		RCLCPP_INFO_STREAM(LOGGER, params_.queries_list[i] << " at frequency " << params_.queries.queries_list_map.at(params_.queries_list[i]).frequency);
		max_frequency = std::max(max_frequency, static_cast<int>(params_.queries.queries_list_map.at(params_.queries_list[i]).frequency));
		ss_gen << "/\"DF" << i << "?\",\"?\"";
		formQuery(params_.queries_list[i], ss_gen);
		ss_gen << "# " << 1000 / params_.queries.queries_list_map.at(params_.queries_list[i]).frequency << "_";		
	}

	ser_.write(ss_gen.str()); // Send commands and queries
	ser_.flush();
	RCLCPP_INFO_STREAM(LOGGER, ss_gen.str());
	serial_read_pub_ = this->create_publisher<std_msgs::msg::String>("read_serial", rclcpp::SystemDefaultsQoS());

	RCLCPP_INFO_STREAM(LOGGER, "Max frequency " << max_frequency);

	this->create_wall_timer(std::chrono::duration<double>{1.0/max_frequency}, std::bind(&RoboteqDriver::queryCallback, this));
}

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