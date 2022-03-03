// #include <serial/serial.h>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_srvs/SetBool.h>
#include <iostream>
#include <sstream>
#include <typeinfo>
#include <deque>
#include <numeric>
#include <roboteq_motor_controller_driver/channel_values.h>

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
        return std::accumulate(data.begin(), data.end(), 0) / ((double)data.size());
    }

    void reset_data( double val)
    {
        std::fill(data.begin(), data.end(), val);
    }

    std::deque<double> get_data()
    {
        return data;
    }

    void print_data()
    {
        for (auto it = data.begin(); it != data.end(); it++)
            std::cout << *it << std::endl;
    }

private:
    std::deque<double> data;
};

class MovePrismaticJoints
{
public:
    MovePrismaticJoints(ros::NodeHandle n_, int hz = 2, int time_delay_ = 25, int current_threshold_ = 4, int rpm_cmd_ = 300, int mva_len_ = 7, double mva_val_ = 200) 
    : n(n_), time_delay(time_delay_), current_threshold(current_threshold_), rpm_cmd(rpm_cmd_), mva_len(mva_len_), mva_val(mva_val_)
    {
        // move_prismatic = n.advertiseService("move_prismatic", &MovePrismaticJoints::movePrismatic, this);
        prismatic_cmd_pub = n.advertise<std_msgs::Int16>("prism_vel", 10);
        motor_amp_sub = n.subscribe<roboteq_motor_controller_driver::channel_values>("motor_amps", 1, &MovePrismaticJoints::motor_amp_callback, this);
        loop_rate = new ros::Rate(hz);
        mva_array = new Moving_Average(mva_len, mva_val) ;
    }

    bool movePrismatic(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        bool timer_pass = 0;
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


        while (ros::ok())
        {
            (ros::Time::now().toSec() - timer_start < time_delay) ? timer_pass = 1 : timer_pass = 0;
            std_msgs::Int16 msg;

            if ((mowing_plate_cmd == 1) && (timer_pass))
            {
                msg.data = rpm_cmd;
                double res = mva_array->update(total_current);
                // std::cout<<"Result: "<<res<<std::endl;
                // std::cout<<"Sec: "<<ros::Time::now().toSec() - timer_start<<std::endl;
                if ((res < current_threshold)||!timer_pass)
                {
                    mowing_plate_cmd = 0;
                    state = 'UP';
                    ROS_INFO_STREAM("Stop moving deck");
                }
            }
            else if ((mowing_plate_cmd == -1) && (timer_pass))
            {
                msg.data = -rpm_cmd;
                double res = mva_array->update(total_current);
                // std::cout<<"Result: "<<res<<std::endl;
                // std::cout<<"Sec: "<<ros::Time::now().toSec() - timer_start<<std::endl;
                if ((res < current_threshold)||!timer_pass)
                {
                    mowing_plate_cmd = 0;
                    state = 'DOWN';
                    ROS_INFO_STREAM("Stop moving deck");
                }
            }
            else
            {
                mva_array->reset_data(mva_val);
                // std::cout<<"Result: "<<res<<std::endl;
                msg.data = 0;
                prismatic_cmd_pub.publish(msg);
                loop_rate->sleep();
                if (timer_pass)
                {
                    res.success = true;
                    res.message = state;
                    return true;
                }
                else
                {
                    res.success = false;
                    res.message = 'UNDEFINED';
                    return false;                    
                }
            }
            prismatic_cmd_pub.publish(msg);
            ros::spinOnce();
            loop_rate->sleep();
        }
    }

private:
    void motor_amp_callback(roboteq_motor_controller_driver::channel_values msg)
    {
        // total_current is the sum of the absolute current in 100 mA
        total_current = abs(msg.value[0]) + abs(msg.value[1]);
    }

    ros::ServiceServer move_prismatic;
    ros::Publisher prismatic_cmd_pub;
    ros::Subscriber motor_amp_sub;
    ros::Rate *loop_rate;
    ros::NodeHandle n;
    
    int time_delay;
    int current_threshold; // e.g 4 -> 400 mA
    int rpm_cmd;

    Moving_Average *mva_array;
    int mva_len;
    double mva_val;

    int total_current = 0;
    int mowing_plate_cmd = 0;
    double timer_start;
    std::string state;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_prismatic_service");
    ros::NodeHandle n;
    MovePrismaticJoints mvp(n);
    ros::ServiceServer ss = n.advertiseService("move_prismatic", &MovePrismaticJoints::movePrismatic, &mvp);

    ros::spin();
    return 0;
}
