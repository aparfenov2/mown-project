#ifndef COSTMAP_2D_MY_TESTING_HELPER_H
#define COSTMAP_2D_MY_TESTING_HELPER_H

#include<costmap_2d_my/cost_values.h>
#include<costmap_2d_my/costmap_2d.h>
#include <costmap_2d_my/static_layer.h>
#include <costmap_2d_my/obstacle_layer.h>
#include <costmap_2d_my/inflation_layer.h>

#include <sensor_msgs/point_cloud2_iterator.h>

const double MAX_Z(1.0);

char printableCost(unsigned char cost)
{
  switch (cost)
  {
  case costmap_2d_my::NO_INFORMATION: return '?';
  case costmap_2d_my::LETHAL_OBSTACLE: return 'L';
  case costmap_2d_my::INSCRIBED_INFLATED_OBSTACLE: return 'I';
  case costmap_2d_my::FREE_SPACE: return '.';
  default: return '0' + (unsigned char) (10 * cost / 255);
  }
}

void printMap(costmap_2d_my::Costmap2D& costmap)
{
  printf("map:\n");
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      printf("%4d", int(costmap.getCost(j, i)));
    }
    printf("\n\n");
  }
}

unsigned int countValues(costmap_2d_my::Costmap2D& costmap, unsigned char value, bool equal = true)
{
  unsigned int count = 0;
  for (int i = 0; i < costmap.getSizeInCellsY(); i++){
    for (int j = 0; j < costmap.getSizeInCellsX(); j++){
      unsigned char c = costmap.getCost(j, i);
      if ((equal && c == value) || (!equal && c != value))
      {
        count+=1;
      }
    }
  }
  return count;
}

void addStaticLayer(costmap_2d_my::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  costmap_2d_my::StaticLayer* slayer = new costmap_2d_my::StaticLayer();
  layers.addPlugin(boost::shared_ptr<costmap_2d_my::Layer>(slayer));
  slayer->initialize(&layers, "static", &tf);
}

costmap_2d_my::ObstacleLayer* addObstacleLayer(costmap_2d_my::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  costmap_2d_my::ObstacleLayer* olayer = new costmap_2d_my::ObstacleLayer();
  olayer->initialize(&layers, "obstacles", &tf);
  layers.addPlugin(boost::shared_ptr<costmap_2d_my::Layer>(olayer));
  return olayer;
}

void addObservation(costmap_2d_my::ObstacleLayer* olayer, double x, double y, double z = 0.0,
                    double ox = 0.0, double oy = 0.0, double oz = MAX_Z){
  sensor_msgs::PointCloud2 cloud;
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(1);
  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
  *iter_x = x;
  *iter_y = y;
  *iter_z = z;

  geometry_msgs::Point p;
  p.x = ox;
  p.y = oy;
  p.z = oz;

  costmap_2d_my::Observation obs(p, cloud, 100.0, 100.0);  // obstacle range = raytrace range = 100.0
  olayer->addStaticObservation(obs, true, true);
}

costmap_2d_my::InflationLayer* addInflationLayer(costmap_2d_my::LayeredCostmap& layers, tf2_ros::Buffer& tf)
{
  costmap_2d_my::InflationLayer* ilayer = new costmap_2d_my::InflationLayer();
  ilayer->initialize(&layers, "inflation", &tf);
  boost::shared_ptr<costmap_2d_my::Layer> ipointer(ilayer);
  layers.addPlugin(ipointer);
  return ilayer;
}


#endif  // COSTMAP_2D_MY_TESTING_HELPER_H
