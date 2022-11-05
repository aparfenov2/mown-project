#include <color_layer/color_layer.h>
#include <pluginlib/class_list_macros.h>

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
        nh.param("grid_map_updates_topic", grid_map_updates_topic);
        grid_sub_ = g_nh.subscribe(grid_map_updates_topic, 1, &ColorLayer::onGridMapUpdate, this);

        matchSize();

        dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
            &ColorLayer::reconfigureCB, this, _1, _2);
        dsrv_->setCallback(cb);
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
        // touch(mark_x, mark_y, min_x, min_y, max_x, max_y);
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
        //         if (costmap_[index] == NO_INFORMATION)
        //             continue;
        //         master_grid.setCost(i, j, costmap_[index]);
        //     }
        // }
        updateWithMax(&master_grid, min_i, min_j, max_i, max_j);
    }


void ColorLayer::onGridMapUpdate(const grid_map_msgs::GridMap& message) {
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

    // if (!grid_map::GridMapRosConverter::fromMessage(message, map)) {
    //     ROS_WARN("failed to read grid_map_msgs::GridMap");
    //     return;
    // }
    ROS_INFO_THROTTLE(5.0, "received GridMap update");
}

} // end namespace
