#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <cmath>

double wheelbase = 0.1;  // Distance between the two wheels
double wheel_radius = 0.04;  // Radius of the robot wheels

double left_wheel_vel = 0.0;
double right_wheel_vel = 0.0;

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel_msg) {
    // Extract linear and angular velocities from cmd_vel message
    double linear_vel = cmd_vel_msg->linear.x;
    double angular_vel = cmd_vel_msg->angular.z;

    // Calculate left and right wheel velocities based on differential drive model
    left_wheel_vel = (linear_vel - angular_vel * wheelbase / 2.0) / wheel_radius;
    right_wheel_vel = (linear_vel + angular_vel * wheelbase / 2.0) / wheel_radius;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "odometry_publisher");
    ros::NodeHandle nh;

    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
    tf::TransformBroadcaster odom_broadcaster;

    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, cmdVelCallback);

    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();

    ros::Rate r(10.0);  // Adjust the rate as needed

    while (ros::ok()) {
        ros::spinOnce();
        current_time = ros::Time::now();
        double dt = (current_time - last_time).toSec();

        // Calculate robot velocities based on wheel velocities
        double linear_vel = (left_wheel_vel + right_wheel_vel) / 2.0;
        double angular_vel = (right_wheel_vel - left_wheel_vel) / wheelbase;

        // Update robot pose
        x += linear_vel * cos(th) * dt;
        y += linear_vel * sin(th) * dt;
        th += angular_vel * dt;

        // Create and broadcast the odom transform
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;
        odom_broadcaster.sendTransform(odom_trans);

        // Create and publish the odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.twist.twist.linear.x = linear_vel;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = angular_vel;
        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();
    }

    return 0;
}
