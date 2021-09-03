#include <pluginlib/class_list_macros.h>
#include <costmap_2d/costmap_layer.h>

namespace full_coverage_path_planner
{

    using costmap_2d::FREE_SPACE;
    using costmap_2d::LETHAL_OBSTACLE;
    using costmap_2d::NO_INFORMATION;

    class CoverageLayer : public costmap_2d::CostmapLayer
    {
    public:
        bool activated_;
        double min_x_, min_y_, max_x_, max_y_;

        CoverageLayer()
        {
            activated_ = true;
            default_value_ = NO_INFORMATION;
        }

        void onInitialize()
        {
            matchSize();
        }

        void updateBounds(double origin_x, double origin_y, double origin_yaw, double *min_x, double *min_y,
                          double *max_x, double *max_y)
        {
            if (!activated_)
                return;
            *min_x = min_x_;
            *min_y = min_y_;
            *max_x = max_x_;
            *max_y = max_y_;
        }

        void updateCosts(costmap_2d::Costmap2D &master_grid, // NOLINT (runtime/references)
                         int min_i, int min_j, int max_i, int max_j)
        {
            if (!activated_)
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

        void activate()
        {
            activated_ = true;
            onInitialize();
        }

        void deactivate()
        {
            activated_ = false;
            // fill the grid with the default_value_ to overwrite the polygon drawing
            resetMaps();
            // shutdown services
        }

        void reset()
        {
            memset(costmap_, default_value_, size_x_ * size_y_ * sizeof(unsigned char));
        }
    };
}

PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::CoverageLayer, costmap_2d::Layer)
