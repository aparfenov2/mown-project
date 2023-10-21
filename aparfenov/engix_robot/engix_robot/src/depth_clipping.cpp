#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/OccupancyGrid.h>

#include "engix_robot/GridMap2D.h"

ros::Publisher image_pub;
double far_threshold_default = 2.5;
double far_threshold;
cv_bridge::CvImagePtr ptr;

void far_clipping_callback(const std_msgs::Float32 &clippingDistPtr)
{
  far_threshold = clippingDistPtr.data;
}

bool updateMap(gridmap_2d::GridMap2DPtr map){
  try
  {
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "CV_8UC1", map->binaryMap()).toImageMsg();
    image_pub.publish(msg);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return false;
  }
}

void mapCallback(const nav_msgs::OccupancyGridConstPtr &occupancy_map){
    gridmap_2d::GridMap2DPtr map(new gridmap_2d::GridMap2D(occupancy_map));
    updateMap(map);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "clipping_node");
  ros::NodeHandle nh("~");
  nh.param<double>("clipping_distance", far_threshold, far_threshold_default);
  ros::Subscriber clipping_distance_sub = nh.subscribe("/clipping/distance", 1, far_clipping_callback);
  ros::Subscriber map_subscripber = nh.subscribe<nav_msgs::OccupancyGrid>("/move_base/local_costmap/costmap", 1, mapCallback);

  image_pub = nh.advertise<sensor_msgs::Image>("/clipping/output", 1);
  ros::spin();
  return 0;
}
