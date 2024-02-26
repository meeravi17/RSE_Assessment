#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <boost/bind/bind.hpp>

// Define BOOST_BIND_GLOBAL_PLACEHOLDERS before including any Boost headers
#define BOOST_BIND_GLOBAL_PLACEHOLDERS
#include <boost/bind/placeholders.hpp>

using namespace boost::placeholders;


class CmdVelRemapper
{
public:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg);
    ros::Publisher remapped_cmd_vel_pub_;
    std::string remapped_topic_;
};

void CmdVelRemapper::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    remapped_cmd_vel_pub_.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmd_vel_remapper_node");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    CmdVelRemapper cmd_vel_remapper;
    private_nh.param<std::string>("remapped_topic", cmd_vel_remapper.remapped_topic_, "/steer_bot/ackermann_steering_controller/cmd_vel");

    // Use boost::placeholders::_1 explicitly
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(&CmdVelRemapper::cmdVelCallback, &cmd_vel_remapper, boost::placeholders::_1));

    cmd_vel_remapper.remapped_cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_remapper.remapped_topic_, 1);

    ros::spin();

    return 0;
}
