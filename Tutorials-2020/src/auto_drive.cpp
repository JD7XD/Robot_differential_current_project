#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <cstdlib>

bool obstacle_detected = false;
ros::Publisher left_wheel_pub;
ros::Publisher right_wheel_pub;

    std_msgs::Float64 left_wheel_cmd;
    std_msgs::Float64 right_wheel_cmd;


void rotateRobot() {
    int i = 1;
    int x = rand()%100;
    if(x>50){i = -1;}
    left_wheel_cmd.data = i*2;  // Rotate left
    right_wheel_cmd.data = -1*i*2;  // Rotate right
    left_wheel_pub.publish(left_wheel_cmd);
    right_wheel_pub.publish(right_wheel_cmd);
    ros::Duration(2.0).sleep();
}

void moveForward() {
    left_wheel_cmd.data = 5;  
    right_wheel_cmd.data = 5;  
    left_wheel_pub.publish(left_wheel_cmd);
    right_wheel_pub.publish(right_wheel_cmd);
}
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{   
    if (obstacle_detected) {
        if (scan->ranges[scan->ranges.size() / 2] > 1.0) {
            obstacle_detected = false;
            ROS_INFO("Object Detected");
            moveForward();  
        }
    }
else {
        double fourfive_angle = 0 * (M_PI / 180.0);
        int num_points = scan->ranges.size();
        int index_L = static_cast<int>((fourfive_angle - scan->angle_min) / scan->angle_increment);
        if (index_L >= 0 && index_L < num_points) {
            double distance_L = scan->ranges[index_L];
            if (!std::isinf(distance_L) && distance_L < 1.3) {
                // Object detected at the desired angle
                ROS_INFO("Object detected at %.2f meters at index %d. Rotating...", distance_L, index_L);
                obstacle_detected = true;
                rotateRobot();
            }
        }
    }
      
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "travel");
    ros::NodeHandle nh;

    // Create a subscriber for the Hokuyo LIDAR data
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidarCallback);

    // Create a publisher for the minimum distance
   
    left_wheel_pub = nh.advertise<std_msgs::Float64>("/left_arm_controller/command", 1);
    right_wheel_pub = nh.advertise<std_msgs::Float64>("/right_arm_controller/command", 1);
    
    ros::spin();

    return 0;
}
