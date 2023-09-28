#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int16.hpp>
#include <geometry_msgs/msg/twist.hpp>

template <typename T> float sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

class RoboteqDriver : public rclcpp::Node
{

public:
	RoboteqDriver();
	~RoboteqDriver();

private:
	void connect();
	
	
	
	std::string port;
	int32_t baud;
	bool safe_speed;
	std::string channel_mode;
	std::string motor_type;
	std::string motor_1_type;
	std::string motor_2_type;
	double track_width;
	double max_vel_x;
	double max_vel_ang;
	double wheel_circumference;
	
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd_vel_channel_1_sub;
	rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr cmd_vel_channel_2_sub;
	
	
	
	serial::Serial ser_;


	
	
	
	
	int max_rpm = 3500;
	double reduction_ratio;
	ros::Publisher read_publisher;
	

	
	int rate;
	ros::Time previous_time;

	ros::NodeHandle nh;

	// added in changes
	ros::NodeHandle nh_priv_;
	std::vector<int> f_list; // a list of frequencies for the queries to be published
	std::vector<ros::Publisher> query_pub_;
	ros::NodeHandle nh_;
	ros::Timer timer_pub_;
	
	std::mutex locker;
	ros::Publisher serial_read_pub_;

	ros::NodeHandle n;
	ros::ServiceServer configsrv;
	ros::ServiceServer commandsrv;
	ros::ServiceServer maintenancesrv;

	std::vector<int> cum_query_size;

	void queryCallback(const ros::TimerEvent &);

	void formQuery(std::string,
				   std::map<std::string, std::string> &,
				   std::vector<ros::Publisher> &,
				   std::stringstream &);

	
	
	void rpm_mapping(const double &right_speed, const double &left_speed, double &right_speed_cr, double &left_speed_cr);
	void skid_steering_vel_callback(const geometry_msgs::Twist &msg);
	void dual_vel_callback(const std_msgs::Int16 &msg);
	void channel_1_vel_callback(const std_msgs::Int16 &msg);
	void channel_2_vel_callback(const std_msgs::Int16 &msg);
	bool configservice(roboteq_motor_controller_driver::config_srv::Request &request, roboteq_motor_controller_driver::config_srv::Response &response);
	bool commandservice(roboteq_motor_controller_driver::command_srv::Request &request, roboteq_motor_controller_driver::command_srv::Response &response);
	bool maintenanceservice(roboteq_motor_controller_driver::maintenance_srv::Request &request, roboteq_motor_controller_driver::maintenance_srv::Response &response);
	void initialize_services();
	void run();
};
