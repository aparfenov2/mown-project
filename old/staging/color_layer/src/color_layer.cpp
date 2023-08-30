#include <color_layer/color_layer.h>
#include <pluginlib/class_list_macros.h>
#include <grid_map_core/iterators/GridMapIterator.hpp>
#include <random>

PLUGINLIB_EXPORT_CLASS(color_layer::ColorLayer, costmap_2d::Layer)

using costmap_2d::LETHAL_OBSTACLE;
using costmap_2d::NO_INFORMATION;

namespace color_layer
{

    ColorLayer::ColorLayer() {}

    void ColorLayer::onInitialize()
    {
        ROS_INFO("name_ %s", name_.c_str());
        ros::NodeHandle nh("~/" + name_), g_nh;
        current_ = true;
        default_value_ = NO_INFORMATION;
        last_hit_ = 0.0;
        last_update_s_ = 0.0;

        log_name_ = name_;

        std::string grid_map_updates_topic;
        if (!nh.getParam("grid_map_updates_topic", grid_map_updates_topic)) {
            ROS_FATAL_STREAM(log_name_ << ": need grid_map_updates_topic");
        }
        // int grass_color;
        nh.param<int>("grass_color", (int&)grass_color_, 0x00ff2600);
        nh.param<int>("road_color", (int&)road_color_, 0x00200000);
        nh.param<bool>("use_road_color", use_road_color_, true);
        nh.param<bool>("black_is_obstacle", black_is_obstacle_, true);
        // grass_color_ = (unsigned long)grass_color;
        int non_grass_cost;
        nh.param<int>("non_grass_cost", non_grass_cost, 200);
        non_grass_cost_ = (unsigned char) non_grass_cost;

        grid_sub_ = g_nh.subscribe(grid_map_updates_topic, 1, &ColorLayer::onGridMapUpdate, this);

        matchSize();

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &ColorLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
        ROS_INFO_STREAM(log_name_ << ": ColorLayer initialized");
    }

    void ColorLayer::reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
    {
        enabled_ = config.enabled;
    }

    void ColorLayer::updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x,
                                  double *min_y, double *max_x, double *max_y)
    {
        if (layered_costmap_->isRolling()) {
            updateOrigin(robot_x - getSizeInMetersX() / 2, robot_y - getSizeInMetersY() / 2);
        }
        if (!enabled_)
            return;

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

        updateWithMax(master_grid, min_i, min_j, max_i, max_j);
    }


void ColorLayer::onGridMapUpdate(const grid_map_msgs::GridMap& message) {
    grid_map::GridMap map;

    if (!grid_map::GridMapRosConverter::fromMessage(message, map)) {
        ROS_WARN_STREAM(log_name_ << ": failed to read grid_map_msgs::GridMap");
        return;
    }

    // ROS_INFO("received GridMap update");
    int color_matches = 0;
    // int min_mx = 1000, min_my = 1000, max_mx = -1000, max_my = -1000;
    std::set<unsigned long> available_colors;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {


        union {
            unsigned long longColor;
            float floatColor;
        } colors;
        colors.floatColor = map.at("color", *it);
        unsigned char cost = 0;
        available_colors.insert(colors.longColor);

        if (!map.isValid(*it, "color")) {
            cost = costmap_2d::NO_INFORMATION;

        }
        if (!use_road_color_) {
            if ((colors.longColor & 0x00ffffff) != (grass_color_ & 0x00ffffff)) {
                cost = non_grass_cost_;
                color_matches++;
            }
        } else {
            if ((colors.longColor & 0x00ffffff) == (road_color_ & 0x00ffffff)) {
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
            ROS_FATAL_STREAM(log_name_ << ": illegal map iterator");
        }

        int mx, my;
        worldToMapEnforceBounds(pos.x(), pos.y(), mx, my);
        setCost(mx, my, cost);
    }

    auto now = ros::Time::now().toSec();
    const double rate = 5.0;
    if (last_hit_ + rate <= now) {
        last_hit_ = now;
        ROS_INFO((log_name_ + ": received GridMap update  (freq %f hz) color_matches %d").c_str(), 1.0/(now - last_update_s_), color_matches);
        if (!color_matches) {
            ROS_INFO((log_name_ + ": grass_color %#08x road_color %#08x use_road_color %d").c_str(), grass_color_, road_color_, use_road_color_);
            ROS_INFO((log_name_ + ": available_colors").c_str());
            for (auto const& color : available_colors) {
                ROS_INFO((log_name_ + ": color %#08x color & 0x00ffffff %#08x, == road_color_ %d").c_str(), color, color & 0x00ffffff, (color & 0x00ffffff) == (road_color_ & 0x00ffffff));
            }
        }
    }
    last_update_s_ = now;
}

} // end namespace
