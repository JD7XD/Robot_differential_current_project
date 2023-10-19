#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>  
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

ros::Publisher hello;
ros::Publisher left_wheel_pub;
ros::Publisher right_wheel_pub;

void lidarCallback(const std_msgs::Float32::ConstPtr& min)
{
        std_msgs::Float64 left_wheel_cmd;
        std_msgs::Float64 right_wheel_cmd;
    if (min->data<2.0){
        left_wheel_cmd.data = -5;
        right_wheel_cmd.data = -5;
        left_wheel_pub.publish(left_wheel_cmd);
        right_wheel_pub.publish(right_wheel_cmd);
        
    }
    if (min->data>2.0){
        left_wheel_cmd.data = -2;
        right_wheel_cmd.data = 0;
        left_wheel_pub.publish(left_wheel_cmd);
        right_wheel_pub.publish(right_wheel_cmd);
    }
    std_msgs::String msg;
    msg.data = "Working perfect";
    hello.publish(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "stop");
    ros::NodeHandle nh;

    // Create a subscriber for the Hokuyo LIDAR data
    ros::Subscriber dis_sub = nh.subscribe<std_msgs::Float32>("/min_distance", 1, lidarCallback);

    // Create a publisher for the minimum distance
    hello = nh.advertise<std_msgs::String>("Object_", 1);
    left_wheel_pub = nh.advertise<std_msgs::Float64>("/left_arm_controller/command", 1);
    right_wheel_pub = nh.advertise<std_msgs::Float64>("/right_arm_controller/command", 1);
    ros::spin();

    return 0;
}
