#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_broadcaster.h"
#include "steer_bot_navigation/OdometryResetRequest.h"
#include <mutex>

class AckermanController
{
private:
    ros::NodeHandle nh;
    ros::Subscriber cmd_vel_sub;
    ros::Publisher odom_pub;
    ros::ServiceServer reset_odometry_service;

    ros::Time current_time, last_time;
    geometry_msgs::Twist cmd_vel;
    std::mutex mutex_;

    double wheel_base;
    std::string cmd_vel_topic, odom_topic;
    double loop_rate;
    double x, y, yaw;
    tf::TransformListener tf_listener;
    tf::StampedTransform transform;

public:
    AckermanController() : nh("~")
    {
        ros::param::param<double>("~wheel_base", wheel_base, 1.0);
        ros::param::param<std::string>("~cmd_vel_topic", cmd_vel_topic, "/steer_bot/ackermann_controller/cmd_vel");
        ros::param::param<std::string>("~odom_topic", odom_topic, "/steer_bot/ackermann_controller/odom");
        ros::param::param<double>("~loop_rate", loop_rate, 20.0);

        cmd_vel_sub = nh.subscribe(cmd_vel_topic, 10, &AckermanController::cmdVelCallback, this);
        odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);

        // Add the service server for resetting odometry
        reset_odometry_service = nh.advertiseService("reset_odometry", &AckermanController::odomServiceCallback, this);

        current_time = ros::Time::now();
        last_time = current_time;

        x = 0.0;
        y = 0.0;
        yaw = 0.0;
    }

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        cmd_vel.linear = msg->linear;
        cmd_vel.angular = msg->angular;
    }

    double getLoopRate()
    {
        return loop_rate;
    }

    void odometryEstimator()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        current_time = ros::Time::now();
        double v = cmd_vel.linear.x;
        double delta = cmd_vel.angular.z;
        double R = wheel_base / tan(delta);
        double omega = v / R;
        double dt = (current_time - last_time).toSec();

        // if (fabs(delta) < 1e-6)
        // {
        //     ROS_ERROR("[Odometry Publisher] Steering angle is too small. Avoiding division by zero.");
        //     return;
        // }

        x += v * cos(yaw) * dt;
        y += v * sin(yaw) * dt;
        yaw += omega * dt;

        last_time = current_time;
        publishVelocity(x, y, v, omega, yaw);
    }

    void publishVelocity(double x, double y, double v, double omega, double yaw)
    {
        nav_msgs::Odometry odom_msg;
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";

        try
        {
            tf_listener.waitForTransform("odom", "base_link", current_time, ros::Duration(1.0));
            tf_listener.lookupTransform("odom", "base_link", current_time, transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("[Odometry Publisher] Unable to get transformation from 'odom' to 'base_link': %s", ex.what());
            return;
        }

        odom_msg.pose.pose.position.x = x;
        odom_msg.pose.pose.position.y = y;
        odom_msg.pose.pose.position.z = 0.0;

        tf::Quaternion quaternion;
        quaternion.setRPY(0, 0, yaw);
        tf::quaternionTFToMsg(quaternion, odom_msg.pose.pose.orientation);

        odom_msg.twist.twist.linear.x = v;
        odom_msg.twist.twist.linear.y = 0.0;
        odom_msg.twist.twist.angular.z = omega;
        odom_pub.publish(odom_msg);
    }

    bool odomServiceCallback(steer_bot_navigation::OdometryResetRequest::Request &req,
                             steer_bot_navigation::OdometryResetRequest::Response &res)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (req.reset)
        {
            x = y = yaw = cmd_vel.linear.x = cmd_vel.angular.z = 0.0;
            res.result = true;
        }
        else
        {
            ROS_WARN("[Odometry Publisher] Invalid request for odometry reset. Ignoring.");
            res.result = false;
        }
        return true;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ackermann_controller_node");
    AckermanController control_node;

    ros::Rate loop_rate(control_node.getLoopRate());
    while (ros::ok())
    {
        control_node.odometryEstimator();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
