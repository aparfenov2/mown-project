#include <unordered_set>
#include <cmath>

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

    struct MapLocation_hash {
        bool operator () (const costmap_2d::MapLocation &a, const costmap_2d::MapLocation &b) const {
            return a.x == b.x && a.y == b.y;
        }
        size_t operator () (const costmap_2d::MapLocation& point) const {
            size_t xHash = std::hash<int>()(point.x);
            size_t yHash = std::hash<int>()(point.y) << 1;
            return xHash ^ yHash;
        }
    };

    class CoverageLayer : public costmap_2d::CostmapLayer
    {
    public:
        dynamic_reconfigure::Server<costmap_2d::GenericPluginConfig> *dsrv_;
        std::unordered_set<costmap_2d::MapLocation, MapLocation_hash, MapLocation_hash> visited_cells_;
        std::vector<costmap_2d::MapLocation> not_footprint_;
        double least_obstacles_direction_;
        std::vector<costmap_2d::MapLocation> footprint_points_;

        CoverageLayer()
        {
            default_value_ = NO_INFORMATION;
        }

        void onInitialize()
        {
            ros::NodeHandle nh("~/" + name_);
            default_value_ = NO_INFORMATION;
            current_ = true;
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

            footprint_points_ = footprint_to_points(origin_x, origin_y, origin_yaw);
            std::unordered_set<costmap_2d::MapLocation, MapLocation_hash, MapLocation_hash> footprint_set(footprint_points_.begin(), footprint_points_.end());
            visited_cells_.insert(footprint_set.begin(), footprint_set.end());
            not_footprint_.clear();

            // not_footprint_ = visited_cells_ - footprint_set
            std::copy_if(visited_cells_.begin(), visited_cells_.end(), std::back_inserter(not_footprint_),
                [&footprint_set] (costmap_2d::MapLocation needle) { return footprint_set.find(needle) == footprint_set.end(); });

            for (auto &it : not_footprint_)
            {
                visited_cells_.erase(it);
            }

            // avoid placing obstacles in front while moving backwards
            unsigned int origin_mx, origin_my;
            worldToMap(origin_x, origin_y, origin_mx, origin_my);
            // scan in all directions
            least_obstacles_direction_ = origin_yaw;
            int least_cost = INT_MAX;

            for (double phi=origin_yaw; phi < origin_yaw + 2*M_PI; phi += M_PI_4)
            {
                std::vector<costmap_2d::MapLocation> cells;
                costmap_2d::Costmap2D::PolygonOutlineCells cell_gatherer(*this, nullptr, cells);
                double radius = 1.5 * layered_costmap_->getCircumscribedRadius();
                ROS_DEBUG_ONCE("scan radius %f", radius);
                double x1 = origin_x + radius * std::cos(phi);
                double y1 = origin_y + radius * std::sin(phi);

                unsigned int mx1, my1;
                if (!worldToMap(x1, y1, mx1, my1)) {
                    continue;
                }
                raytraceLine(cell_gatherer, origin_mx, origin_my, mx1, my1);
                int cost = 0;
                for (const costmap_2d::MapLocation& p : cells) {
                    cost += getCost(p.x, p.y) == LETHAL_OBSTACLE ? 1 : 0;
                }
                if (cost < least_cost) {
                    least_cost = cost;
                    least_obstacles_direction_ = phi;
                }
            }

            // not_footprint_.erase(
            //     std::remove_if(not_footprint_.begin(), not_footprint_.end(),
            //         [least_obstacles_direction_, origin_mx, origin_my](const costmap_2d::MapLocation& p) -> bool {
            //             int rx = p.x - origin_mx;
            //             int ry = p.y - origin_my;
            //             double q = std::atan2((double)ry , (double)rx);
            //             return least_obstacles_direction_ - M_PI_2 < q && q < least_obstacles_direction_ + M_PI_2;
            //         }),
            //     not_footprint_.end()
            //     );

            // update update bounds
            for (auto &it : not_footprint_)
            {
                double mark_x, mark_y;
                mapToWorld(it.x, it.y, mark_x, mark_y);
                *min_x = std::min(*min_x, mark_x);
                *min_y = std::min(*min_y, mark_y);
                *max_x = std::max(*max_x, mark_x);
                *max_y = std::max(*max_y, mark_y);
            }
            for (auto &it : footprint_points_)
            {
                double mark_x, mark_y;
                mapToWorld(it.x, it.y, mark_x, mark_y);
                *min_x = std::min(*min_x, mark_x);
                *min_y = std::min(*min_y, mark_y);
                *max_x = std::max(*max_x, mark_x);
                *max_y = std::max(*max_y, mark_y);
            }

        }

        void updateCosts(costmap_2d::Costmap2D &master_grid, // NOLINT (runtime/references)
                         int min_i, int min_j, int max_i, int max_j)
        {
            if (!enabled_)
                return;

            for (auto &it : not_footprint_)
            {
                setCost(it.x, it.y, LETHAL_OBSTACLE);
            }
            for (auto &it : footprint_points_) {
                setCost(it.x, it.y, FREE_SPACE);
            }

            updateWithMax(master_grid,  min_i,  min_j,  max_i,  max_j);

        }
    };
}

PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::CoverageLayer, costmap_2d::Layer)
