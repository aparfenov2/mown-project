#include <color_layer/color_layer.h>
#include <pluginlib/class_list_macros.h>
#include <grid_map_core/iterators/GridMapIterator.hpp>

PLUGINLIB_EXPORT_CLASS(color_layer::ColorLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace color_layer
{

    ColorLayer::ColorLayer() {}

    void ColorLayer::onInitialize()
    {
        ros::NodeHandle nh("~/" + name_), g_nh;
        current_ = true;
        default_value_ = NO_INFORMATION;

        std::string grid_map_updates_topic;
        if (!nh.getParam("grid_map_updates_topic", grid_map_updates_topic)) {
            ROS_FATAL("need grid_map_updates_topic");
        }
        // int grass_color;
        nh.param<int>("grass_color", (int&)grass_color_, 0x00ff2600);
        nh.param<int>("road_color", (int&)road_color_, 0x00200000);
        nh.param<bool>("use_road_color", use_road_color_, true);
        nh.param<bool>("black_is_obstacle", black_is_obstacle_, true);        
        // grass_color_ = (unsigned long)grass_color;
        // int non_grass_cost;
        nh.param<int>("non_grass_cost", (int&)non_grass_cost_, 200);
        // non_grass_cost_ = (unsigned char) non_grass_cost;
        
        grid_sub_ = g_nh.subscribe(grid_map_updates_topic, 1, &ColorLayer::onGridMapUpdate, this);

        matchSize();

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &ColorLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        ROS_INFO("ColorLayer initialized");
    }

    void ColorLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    void ColorLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                  double *min_y, double *max_x, double *max_y)
    {
        if (!enabled_)
            return;

        // useExtraBounds(min_x, min_y, max_x, max_y);

        // double mark_x = robot_x + cos(robot_yaw), mark_y = robot_y + sin(robot_yaw);
        // unsigned int mx;
        // unsigned int my;
        // if (worldToMap(mark_x, mark_y, mx, my))
        // {
        //     setCost(mx, my, LETHAL_OBSTACLE);
        // }

        // *min_x = std::min(*min_x, mark_x);
        // *min_y = std::min(*min_y, mark_y);
        // *max_x = std::max(*max_x, mark_x);
        // *max_y = std::max(*max_y, mark_y);
        double wx, wy;
        mapToWorld(0, 0, wx, wy);
        touch(wx, wy, min_x, min_y, max_x, max_y);
        mapToWorld(getSizeInCellsX(), getSizeInCellsY(), wx, wy);
        touch(wx, wy, min_x, min_y, max_x, max_y);
    }

    void ColorLayer::updateCosts(costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i,
                                 int max_j)
    {
        if (!enabled_)
            return;

        // for (int j = min_j; j < max_j; j++)
        // {
        //     for (int i = min_i; i < max_i; i++)
        //     {
        //         int index = getIndex(i, j);
        //         if (costmap_[index] == NO_INFORMATION)ue reference of type ‘int&
        // }
        updateWithAddition(master_grid, min_i, min_j, max_i, max_j);
        // updateWithOverwrite(master_grid, min_i, min_j, max_i, max_j);

        // ROS_INFO("updated master_grid %d %d %d %d",min_i,min_j,max_i, max_j);

    }


void ColorLayer::onGridMapUpdate(const grid_map_msgs::GridMap& message) {
    grid_map::GridMap map;
    // grid_map::GridMap &map = layered_costmap_->getCostmap()->getGridMap();

    // if (message.info.header.frame_id != map.getFrameId()) {
    //     ROS_ERROR_STREAM("grid_map update rejected: expected frame_id: " << map.getFrameId()
    //             << "incoming msg frame id:  " << message.info.header.frame_id);
    //             return;
    // }
    // if (message.info.resolution != map.getResolution() || 
    //     message.info.length_x != map.getLength()(0) || 
    //     message.info.length_y != map.getLength()(1)) {
    //     ROS_ERROR_STREAM("grid_map update rejected: " 
    //         << "\nexpected resolution: " << map.getResolution()
    //         << " length_x_m: " << map.getLength()(0)
    //         << " length_y_m: " << map.getLength()(1)
    //         << "\nreceived resolution:  " << message.info.resolution
    //         << " length_x_m: " << message.info.length_x
    //         << " length_y_m: " << message.info.length_y
    //         );
    //         return;
    // }

    // boost::unique_lock<Costmap2D::mutex_t> lock(*(layered_costmap_->getCostmap()->getMutex()));

    if (!grid_map::GridMapRosConverter::fromMessage(message, map)) {
        ROS_WARN("failed to read grid_map_msgs::GridMap");
        return;
    }

    // ROS_INFO("received GridMap update");
    int color_matches = 0;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        

        union {
            unsigned long longColor;
            float floatColor;
        } colors;
        colors.floatColor = map.at("color", *it);
        // unsigned long r = (colors.longColor & 0x00ff0000) >> 16;
        // unsigned long g = (colors.longColor & 0x0000ff00) >> 8;
        // unsigned long b = colors.longColor & 0x000000ff;
        unsigned char cost = 0;

        if (!map.isValid(*it, "color")) {
            cost = costmap_2d::NO_INFORMATION;
        
        } else if (!use_road_color_) {
            if ((colors.longColor & 0x00ffffff) != grass_color_) {
                cost = non_grass_cost_;
                color_matches++;
            }
        } else {
            if ((colors.longColor & 0x00ffffff) == road_color_) {
                cost = non_grass_cost_;
                color_matches++;
            }
            if (black_is_obstacle_ && (colors.longColor & 0x00ffffff) == 0x000000) {
                cost = non_grass_cost_;
                color_matches++;
            }
            
        }

        grid_map::Position pos;

        if (!map.getPosition(*it, pos)) {
            ROS_FATAL("illegal map iterator, \n\tfile = %s\n\tline=%d\n", __FILE__, __LINE__);
        }

        int mx, my;
        worldToMapEnforceBounds(pos.x(), pos.y(), mx, my);        
        setCost(mx, my, cost);
        // if (cost != costmap_2d::NO_INFORMATION) {
        //     ROS_INFO("mx %d my %d cost %d", mx, my, cost);
        // }

        // ROS_INFO("gx %d gy %d px %f py %f mx %d my %d", (*it)(0), (*it)(1), pos(0), pos(1), mx, my);
    }

    static double last_update_s = 0;
    auto now = ros::Time::now().toSec();
    ROS_INFO_THROTTLE(5.0, "received GridMap update  (freq %f hz) color_matches %d", 1.0/(now - last_update_s), color_matches);
    last_update_s = now;
}

} // end namespace
