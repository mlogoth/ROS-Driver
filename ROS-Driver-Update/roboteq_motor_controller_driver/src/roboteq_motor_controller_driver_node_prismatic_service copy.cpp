#include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_srvs/SetBool.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <deque>
#include <numeric>
#include <roboteq_motor_controller_driver/channel_values.h>

int total_current=0;
int mowing_plate_cmd=0;
double timer_start;

class Moving_Average
{
    public:
    Moving_Average(int size = 10, double val = 10)
    {  
        data.resize(size);
        std::fill(data.begin(), data.end(), val);
    }

    double update(double val)
    {
        data.pop_front();
        data.push_back(val);
        return std::accumulate(data.begin(), data.end(),0)/((double)data.size());
    }
   
    std::deque<double> get_data()
    {
        return data;
    }
   
    void print_data()
    {
        for (auto it = data.begin(); it !=data.end(); it++)
            std::cout<<*it<<std::endl;
    }

    private:
    std::deque<double> data;
};

bool moveprismatic(std_srvs::SetBool::Request  &req, std_srvs::SetBool::Response &res)
{
	timer_start = ros::Time::now().toSec();
	ROS_INFO_STREAM("move_prismatic service called");

	if (req.data == true)
	{
		ROS_INFO_STREAM("Mower deck moving upward");
		mowing_plate_cmd = 1;
	}
	else if (req.data == false)
	{
		
		ROS_INFO_STREAM("Mower deck moving downward");
		mowing_plate_cmd = -1;
	}
	else 
		mowing_plate_cmd = 0;

	res.success = true;
	return true;
}

void motor_amp_callback(roboteq_motor_controller_driver::channel_values msg)
{	
	// total_current is the sum of the absolute current in 100 mA
	total_current = abs(msg.value[0]) + abs(msg.value[1]);
}

int main(int argc, char **argv)
{
	int time_delay = 25;
	int current_threshold = 4; // = 400 mA
	int rpm_cmd = 300;

	Moving_Average mva_array(7,200);

	ros::init(argc, argv, "move_prismatic_service");
	ros::NodeHandle n;

	ros::ServiceServer move_prismatic = n.advertiseService("move_prismatic", moveprismatic);

	ros::Publisher prismatic_cmd_pub = n.advertise<std_msgs::Int16>("prism_vel", 1);

	ros::Subscriber motor_amp_sub = n.subscribe<roboteq_motor_controller_driver::channel_values>("motor_amps", 1, motor_amp_callback);
	
	ros::Rate loop_rate(2);

	while (ros::ok())
	{
		std_msgs::Int16 msg;
		
		if ((mowing_plate_cmd == 1)&&((ros::Time::now().toSec() - timer_start < time_delay)))
		{
			msg.data = rpm_cmd;
			double res = mva_array.update(total_current);
			// std::cout<<"Result: "<<res<<std::endl;
			// std::cout<<"Sec: "<<ros::Time::now().toSec() - timer_start<<std::endl;
			if (res < current_threshold)
			{
				mowing_plate_cmd = 0;
				ROS_INFO_STREAM("Stop moving deck");
			}
		}
		else if ((mowing_plate_cmd == -1)&&((ros::Time::now().toSec() - timer_start < time_delay)))
		{	
			msg.data = -rpm_cmd;
			double res = mva_array.update(total_current);
			// std::cout<<"Result: "<<res<<std::endl;
			// std::cout<<"Sec: "<<ros::Time::now().toSec() - timer_start<<std::endl;
			if (res < current_threshold)
			{
				mowing_plate_cmd = 0;
				ROS_INFO_STREAM("Stop moving deck");
			}
		}
		else
		{	
			double res = mva_array.update(200);
			// std::cout<<"Result: "<<res<<std::endl;
			msg.data = 0;
		}

		prismatic_cmd_pub.publish(msg);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}

