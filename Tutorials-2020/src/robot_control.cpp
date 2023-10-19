#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

ros::Publisher left_wheel_pub;
ros::Publisher right_wheel_pub;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg) {
    // Extract linear and angular velocities from cmd_vel message
    double linear_vel = cmd_vel_msg->linear.x;
    double angular_vel = cmd_vel_msg->angular.z;

    // Calculate wheel speeds for a differential drive robot
    double left_wheel_speed = linear_vel - angular_vel;
    double right_wheel_speed = linear_vel + angular_vel;

    // Publish wheel speed commands
    std_msgs::Float64 left_wheel_cmd;
    std_msgs::Float64 right_wheel_cmd;
    left_wheel_cmd.data = left_wheel_speed;
    right_wheel_cmd.data = right_wheel_speed;
    left_wheel_pub.publish(left_wheel_cmd);
    right_wheel_pub.publish(right_wheel_cmd);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "wheel_controller_node");
    ros::NodeHandle nh;

    // Initialize publishers for left and right wheel commands
    left_wheel_pub = nh.advertise<std_msgs::Float64>("/left_arm_controller/command", 1);
    right_wheel_pub = nh.advertise<std_msgs::Float64>("/right_arm_controller/command", 1);

    // Subscribe to the cmd_vel topic to receive velocity commands
    ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1, cmdVelCallback);

    ros::spin();

    return 0;
}
