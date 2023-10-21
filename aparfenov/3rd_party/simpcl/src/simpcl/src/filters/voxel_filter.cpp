// Downsampling with "Voxel Grid Filter"
// http://pointclouds.org/documentation/tutorials/voxel_grid.php

// Import dependencies
#include <string>
#include <ros/ros.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/voxel_grid.h>

// Definitions
ros::Publisher pub;
std::string subscribed_topic;
std::string published_topic;
double leaf_size_x, leaf_size_y, leaf_size_z;

// callback
void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Create point cloud objects for raw and downsampled_cloud
    pcl::PCLPointCloud2* raw_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(raw_cloud);
    pcl::PCLPointCloud2 downsampled_cloud;

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *raw_cloud);

    // Perform the VoxelGrid Filter
    pcl::VoxelGrid<pcl::PCLPointCloud2> v_filter;
    v_filter.setInputCloud(cloudPtr); // Pass raw_cloud to the filter
    v_filter.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z); // Set leaf size
    v_filter.filter(downsampled_cloud); // Store output data in downsampled_cloud

    // Convert to ROS data type
    sensor_msgs::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(downsampled_cloud, output_cloud);

    pub.publish(output_cloud); // Publish downsampled_cloud to the /cloud_downsampled topic
}

// main
int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "voxel_filter");
    ros::NodeHandle n;

    // Load parameters from launch file
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("subscribed_topic", subscribed_topic, "/point_cloud");
    nh_private.param<std::string>("published_topic", published_topic, "cloud_downsampled");
    nh_private.param<double>("leaf_size_x", leaf_size_x, 0.05);
    nh_private.param<double>("leaf_size_y", leaf_size_y, 0.05);
    nh_private.param<double>("leaf_size_z", leaf_size_z, 0.15);

    // Create Subscriber and listen /zed/point_cloud/cloud_registered topic
    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>(subscribed_topic, 1, cloud_cb);

    // Create Publisher
    pub = n.advertise<sensor_msgs::PointCloud2>(published_topic, 1);

    // Spin
    ros::spin();
    return 0;
}
