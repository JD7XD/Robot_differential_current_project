#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/MapMetaData.h>
#include <slam_gmapping/GMapping.h>
#include <pcl_ros/point_cloud.h>

class ScanToMapNode
{
public:
    ScanToMapNode() : nh("~")
    {
        // Parameters
        nh.param("map_width", map_width, 400);
        nh.param("map_height", map_height, 400);
        nh.param("map_resolution", map_resolution, 0.05);
        nh.param("robot_radius", robot_radius, 0.2);
        nh.param("map_frame", map_frame, std::string("map"));
        nh.param("base_frame", base_frame, std::string("base_link"));

        // Initialize the GMapping object
        gmapping = new GMapping();

        // Set up map metadata
        map_metadata.map_load_time = ros::Time::now();
        map_metadata.resolution = map_resolution;
        map_metadata.width = map_width;
        map_metadata.height = map_height;
        map_metadata.origin = geometry_msgs::PoseStamped();
        map_metadata.origin.pose.orientation.w = 1.0;

        // Initialize the map
        map_data.assign(map_height * map_width, 0);

        // Create publishers
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1);

        // Create subscribers
        scan_sub = nh.subscribe("/scan", 1, &ScanToMapNode::scanCallback, this);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        // Convert LaserScan to PointCloud2
        pcl::PointCloud<pcl::PointXYZ> pc_data;
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            if (scan_msg->range_min <= scan_msg->ranges[i] && scan_msg->ranges[i] <= scan_msg->range_max)
            {
                double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
                double x = scan_msg->ranges[i] * cos(angle);
                double y = scan_msg->ranges[i] * sin(angle);
                pc_data.push_back(pcl::PointXYZ(x, y, 0.0)); // Z is set to 0
            }
        }

        // Create PointCloud2 message
        pcl::toROSMsg(pc_data, pc_msg);
        pc_msg.header = scan_msg->header;

        // Update GMapping with PointCloud2 data
        gmapping->update(pc_msg);

        // Get the map from GMapping
        map_data = gmapping->getMap();

        // Publish the map
        publishMap();
    }

    void publishMap()
    {
        // Create an OccupancyGrid message
        nav_msgs::OccupancyGrid map_msg;
        map_msg.header.stamp = ros::Time::now();
        map_msg.header.frame_id = map_frame;
        map_msg.info = map_metadata;
        map_msg.data = map_data;

        // Publish the map
        map_pub.publish(map_msg);
    }

    void spin()
    {
        ros::Rate rate(10); // 10 Hz
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle nh;
    GMapping* gmapping;
    std::vector<int8_t> map_data;
    nav_msgs::MapMetaData map_metadata;
    ros::Publisher map_pub;
    ros::Subscriber scan_sub;
    double map_resolution, robot_radius;
    int map_width, map_height;
    std::string map_frame, base_frame;
    sensor_msgs::PointCloud2 pc_msg;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "scan_to_map_node");
    ScanToMapNode node;
    node.spin();
    return 0;
}
