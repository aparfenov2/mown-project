#include <set>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_layer.h>
#include <dynamic_reconfigure/server.h>
#include <costmap_2d/GenericPluginConfig.h>

namespace full_coverage_path_planner
{

    using costmap_2d::FREE_SPACE;
    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;

    struct MapLocation_lesser {
        bool operator() (const costmap_2d::MapLocation& a, const costmap_2d::MapLocation& b) const {
            if (a.x < b.x) return true;
            if (a.x > b.x) return false;
            if (a.y < b.y) return true;
            return false;
        }
    };

    class CoverageLayer : public costmap_2d::CostmapLayer
    {
    public:
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
        std::set<costmap_2d::MapLocation, MapLocation_lesser> visited_cells;
        double min_x_, min_y_, max_x_, max_y_;

        CoverageLayer()
        {
            default_value_ = NO_INFORMATION;
        }

        void onInitialize()
        {
            ros::NodeHandle nh("~/" + name_);
            default_value_ = NO_INFORMATION;
            current_ = true;
            min_x_ = min_y_ = max_x_ = max_y_ = 0;
            matchSize();
            dsrv_ = new dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>(nh);
            dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig>::CallbackType cb = boost::bind(
                &CoverageLayer::reconfigureCB, this, _1, _2);
            dsrv_->setCallback(cb);
        }

        void reconfigureCB(costmap_2d::GenericPluginConfig &config, uint32_t level)
        {
            enabled_ = config.enabled;
        }

        std::vector<costmap_2d::MapLocation> footprint_to_points(double origin_x, double origin_y, double origin_yaw)
        {
            std::vector<geometry_msgs::Point> footprint_spec = getFootprint();

            double cos_th = cos(origin_yaw);
            double sin_th = sin(origin_yaw);
            std::vector<geometry_msgs::Point> oriented_footprint;

            for (unsigned int i = 0; i < footprint_spec.size(); ++i)
            {
                geometry_msgs::Point new_pt;
                new_pt.x = origin_x + (footprint_spec[i].x * cos_th - footprint_spec[i].y * sin_th);
                new_pt.y = origin_y + (footprint_spec[i].x * sin_th + footprint_spec[i].y * cos_th);
                oriented_footprint.push_back(new_pt);
            }

            // we assume the polygon is given in the global_frame... we need to transform it to map coordinates
            std::vector<costmap_2d::MapLocation> map_polygon;
            for (unsigned int i = 0; i < oriented_footprint.size(); ++i)
            {
                costmap_2d::MapLocation loc;
                if (!worldToMap(oriented_footprint[i].x, oriented_footprint[i].y, loc.x, loc.y))
                {
                    // truncate footprint polygon to map bounds
                    continue;
                }
                map_polygon.push_back(loc);
            }

            std::vector<costmap_2d::MapLocation> polygon_cells;

            // get the cells that fill the polygon
            convexFillCells(map_polygon, polygon_cells);
            return polygon_cells;
        }

        void updateBounds(double origin_x, double origin_y, double origin_yaw, double *min_x, double *min_y,
                          double *max_x, double *max_y)
        {
            if (!enabled_)
                return;

            std::vector<costmap_2d::MapLocation> footprint_points = footprint_to_points(origin_x, origin_y, origin_yaw);
            std::set<costmap_2d::MapLocation, MapLocation_lesser> footprint_set(footprint_points.begin(), footprint_points.end());
            visited_cells.insert(footprint_set.begin(), footprint_set.end());

            std::set<costmap_2d::MapLocation, MapLocation_lesser> not_footprint;
            std::set_difference(visited_cells.begin(), visited_cells.end(), footprint_set.begin(), footprint_set.end(),
                                std::inserter(not_footprint, not_footprint.end()), 
                                [](const costmap_2d::MapLocation& a, const costmap_2d::MapLocation& b) { return MapLocation_lesser{}(a, b); }
                                );

            for (auto &it : visited_cells)
            {
                double mark_x, mark_y;
                mapToWorld(it.x, it.y, mark_x, mark_y);
                min_x_ = std::min(min_x_, mark_x);
                min_y_ = std::min(min_y_, mark_y);
                max_x_ = std::max(max_x_, mark_x);
                max_y_ = std::max(max_y_, mark_y);
            }

            for (auto &it : not_footprint)
            {
                setCost(it.x, it.y, LETHAL_OBSTACLE);
            }
            visited_cells.erase(not_footprint.begin(), not_footprint.end());

            *min_x = min_x_;
            *min_y = min_y_;
            *max_x = max_x_;
            *max_y = max_y_;
        }

        void updateCosts(costmap_2d::Costmap2D &master_grid, // NOLINT (runtime/references)
                         int min_i, int min_j, int max_i, int max_j)
        {
            if (!enabled_)
                return;
            unsigned char *master = master_grid.getCharMap();

            for (int j = min_j; j < max_j; j++)
            {
                for (int i = min_i; i < max_i; i++)
                {
                    int index = getIndex(i, j);
                    if (master[index] != LETHAL_OBSTACLE && (costmap_[index] == LETHAL_OBSTACLE || costmap_[index] > master[index]))
                        master_grid.setCost(i, j, costmap_[index]);
                }
            }
        }
    };
}

PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::CoverageLayer, costmap_2d::Layer)
