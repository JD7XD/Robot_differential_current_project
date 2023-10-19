#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/Twist.h"

// ros::Publisher left_wheel_pub;
// ros::Publisher right_wheel_pub;

// void chatterCallback(const std_msgs::Float64::ConstPtr& msg)
// {
//     ros::NodeHandle n;
//     ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    
//     geometry_msgs::Twist twist;
//     if (msg->data < 2.5) {
//         double linear_vel = 0;
//         double angular_vel = 0;

//     // Calculate wheel speeds for a differential drive robot
//         double left_wheel_speed = linear_vel - angular_vel;
//         double right_wheel_speed = linear_vel + angular_vel;

//     // Publish wheel speed commands
//         std_msgs::Float64 left_wheel_cmd;
//         std_msgs::Float64 right_wheel_cmd;
//         left_wheel_cmd.data = left_wheel_speed;
//         right_wheel_cmd.data = right_wheel_speed;
//         left_wheel_pub.publish(left_wheel_cmd);
//         right_wheel_pub.publish(right_wheel_cmd);
//     }
//     else {
//       ROS_INFO("No obstacle nearby"); 
//     }
// }






// void chatterCallback(const std_msgs::Float64::ConstPtr& msg) {
//     if (msg->data < 2.5) {
//     double linear_vel = 0;
//     double angular_vel = 0;

//     // Calculate wheel speeds for a differential drive robot
//     double left_wheel_speed = linear_vel - angular_vel;
//     double right_wheel_speed = linear_vel + angular_vel;

//     // Publish wheel speed commands
//     std_msgs::Float64 left_wheel_cmd;
//     std_msgs::Float64 right_wheel_cmd;
//     left_wheel_cmd.data = left_wheel_speed;
//     right_wheel_cmd.data = right_wheel_speed;
//     left_wheel_pub.publish(left_wheel_cmd);
//     right_wheel_pub.publish(right_wheel_cmd);
//     }
//     else{
//       ROS_INFO("No obstance nearby");
//     }
// }

// int main(int argc, char** argv) {
//     ros::init(argc, argv, "objd_det");
//     ros::NodeHandle nh;

//     // Initialize publishers for left and right wheel commands
//     left_wheel_pub = nh.advertise<std_msgs::Float64>("/left_arm_controller/command", 1);
//     right_wheel_pub = nh.advertise<std_msgs::Float64>("/right_arm_controller/command", 1);

//     // Subscribe to the cmd_vel topic to receive velocity commands
//     ros::Subscriber sub = nh.subscribe("min_distance", 500, chatterCallback);

//     ros::spin();

//     return 0;
// }

#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

ros::Publisher cmd_vel_pub;

void chatterCallback(const std_msgs::Float32::ConstPtr& msg)
{
    geometry_msgs::Twist twist;
    if (msg->data < 2.5) {
        // Stop the robot
        twist.linear.x = 0.0;
        twist.angular.z = 0.0;
    } else {
        // You can add logic here for other cases if needed
    }
    
    // Publish the twist message to /cmd_vel
    cmd_vel_pub.publish(twist);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_controller_node");
    ros::NodeHandle nh;

    // Initialize the publisher for /cmd_vel
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Subscribe to the min_distance topic to receive distance data
    ros::Subscriber sub = nh.subscribe("min_distance", 500, chatterCallback);

    ros::spin();

    return 0;
}

