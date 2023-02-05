#ifndef GRID_LAYER_H_
#define GRID_LAYER_H_
#include <ros/ros.h>
#include <costmap_2d/layer.h>
#include <costmap_2d/layered_costmap.h>
#include <costmap_2d/costmap_layer.h>
#include <costmap_2d/GenericPluginConfig.h>
#include <dynamic_reconfigure/server.h>

#include <grid_map_ros/grid_map_ros.hpp>

namespace color_layer
{

    class ColorLayer : public costmap_2d::CostmapLayer
    {
    public:
        ColorLayer();

        virtual void onInitialize();
        virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x,
                                  double *max_y);
        virtual void updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j);

        // virtual void matchSize();

    private:
        std::string log_name_;
        unsigned long grass_color_;
        unsigned long road_color_;
        bool use_road_color_;
        bool black_is_obstacle_;
        unsigned char non_grass_cost_;
        double last_hit_; // instance-aware ROS_LOG_THROTTLE
        double last_update_s_;

        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;

        ros::Subscriber grid_sub_;
        void onGridMapUpdate(const grid_map_msgs::GridMap &message);
    };
}
#endif
