#include "ros/ros.h"
#include <std_msgs/Int16.h>
#include "engix_robot/MapAsImageProvider.h"
#include <tf/transform_listener.h>

using namespace std;

MapAsImageProvider *map_image_provider;
int32_t tile_width;
int32_t tile_height;
std::string parent_frame;
std::string child_frame;
bool draw_robot;
bool publish_full_map;
bool publish_map_tile;

void map_zoom_callback(const std_msgs::Int16 &scale)
{
  map_image_provider->setScale((float)scale.data / 2500);
}

int main(int argc, char **argv)
{
  tf::StampedTransform transform;

  ros::init(argc, argv, "map_to_image_node");
  ros::NodeHandle n("~");
  n.param<int32_t>("tile_width", tile_width, INITIAL_TILE_SIZE_X);
  n.param<int32_t>("tile_height", tile_height, INITIAL_TILE_SIZE_Y);
  n.param<std::string>("parent_frame", parent_frame, "/map");
  n.param<std::string>("child_frame", child_frame, "/base_link");
  n.param<bool>("draw_robot", draw_robot, false);
  n.param<bool>("publish_full_map", publish_full_map, true);
  n.param<bool>("publish_map_tile", publish_map_tile, true);

  ros::Rate loop_rate(50);
  ROS_INFO("Init MapAsImageProvider object");
  map_image_provider = new MapAsImageProvider(n, tile_width, tile_height, draw_robot, publish_full_map, publish_map_tile);
  ros::Subscriber map_zoom_sub = n.subscribe("/map_zoom", 1, map_zoom_callback);
  tf::TransformListener listener;
  tfScalar yaw, pitch, roll;
  tf::Matrix3x3 *robot_rot_matrix;

  while (ros::ok())
  {
    ros::spinOnce();
    try
    {
      listener.lookupTransform(parent_frame, child_frame, ros::Time(0), transform);
      robot_rot_matrix = new tf::Matrix3x3(transform.getRotation());
      robot_rot_matrix->getEulerYPR(yaw, pitch, roll);
      map_image_provider->updateRobotPosition(transform.getOrigin().getX(), transform.getOrigin().getY(), yaw);
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("Could not get transform between '%s' and '%s', will retry every second", parent_frame.c_str(), child_frame.c_str());
      ros::Duration(1.0).sleep();
    }

    loop_rate.sleep();
    if (publish_full_map)
    {
      map_image_provider->publishFullMap();
    }

    if (publish_map_tile)
    {
      map_image_provider->publishMapTile();
    }
  }
  return 0;
}
