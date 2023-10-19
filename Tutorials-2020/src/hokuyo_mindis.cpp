#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float32.h>

// Declare distance_pub as a global variable
ros::Publisher distance_pub;

// Callback function to process LIDAR data
void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // Process LIDAR data and find the minimum range
    float min_range = std::numeric_limits<float>::max();
    for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
    {
        if (!std::isnan(scan_msg->ranges[i]) && scan_msg->ranges[i] < min_range)
        {
            min_range = scan_msg->ranges[i];
        }
    }

    // Publish the minimum range as a Float32 message
    std_msgs::Float32 distance_msg;
    distance_msg.data = min_range;
    distance_pub.publish(distance_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "hokuyo_listener");
    ros::NodeHandle nh;

    // Create a subscriber for the Hokuyo LIDAR data
    ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 1, lidarCallback);

    // Create a publisher for the minimum distance
    distance_pub = nh.advertise<std_msgs::Float32>("min_distance", 1);

    ros::spin();

    return 0;
}
