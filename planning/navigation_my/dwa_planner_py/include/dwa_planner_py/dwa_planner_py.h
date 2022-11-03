#ifndef __GRIDMAP_H__
#define __GRIDMAP_H__

#include <grid_map_ros/grid_map_ros.hpp>

class GridMap {
public:
    GridMap(const std::vector<std::string>& layers);
    void update(grid_map_msgs::GridMap message);
    bool getIndex(int x, int y, int cx, int cy);
    float at(int cx, int cy);
    void getLayer(std::string name, std::vector<std::vector<float>> &data);
};

#endif // __GRIDMAP_H__