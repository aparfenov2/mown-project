
#include <costmap_2d_my/grid_costmap.h>

using namespace costmap_2d_my;

unsigned int GridCostmap2D::getSizeInCellsX() const
{
  return costmap_.getSize()(0);
}

unsigned int GridCostmap2D::getSizeInCellsY() const
{
  return costmap_.getSize()(1);
}

double GridCostmap2D::getSizeInMetersX() const
{
  return (costmap_.getSize()(0) - 1 + 0.5) * costmap_.getResolution();
}

double GridCostmap2D::getSizeInMetersY() const
{
  return (costmap_.getSize()(1) - 1 + 0.5) * costmap_.getResolution();
}

double GridCostmap2D::getOriginX() const
{
  return costmap_.getPosition().x();
}

double GridCostmap2D::getOriginY() const
{
  return costmap_.getPosition().y();
}

double GridCostmap2D::getResolution() const
{
  return costmap_.getResolution();
}

void GridCostmap2D::resizeMap(unsigned int size_x, unsigned int size_y, double resolution,
                          double origin_x, double origin_y)
{
    boost::unique_lock<mutex_t> lock(*access_);
    costmap_.setGeometry(grid_map::Length(size_x * resolution, size_y * resolution), 
        resolution, grid_map::Position(origin_x, origin_y));
}

void GridCostmap2D::updateOrigin(double new_origin_x, double new_origin_y)
{
    costmap_.setPosition(grid_map::Position(new_origin_x, new_origin_y));
}

void GridCostmap2D::resetMap(unsigned int x0, unsigned int y0, unsigned int xn, unsigned int yn)
{
  boost::unique_lock<mutex_t> lock(*(access_));
  costmap_.clearAll(); // TODO: clear only specific area
//   unsigned int len = xn - x0;
//   for (unsigned int y = y0 * size_x_ + x0; y < yn * size_x_ + x0; y += size_x_)
//     memset(costmap_ + y, default_value_, len * sizeof(unsigned char));
}

void GridCostmap2D::worldToMapEnforceBounds(double wx, double wy, int& mx, int& my) const
{
    double origin_x_, origin_y_, resolution_, size_x_, size_y_;
    auto pos = costmap_.getPosition();
    auto size = costmap_.getSize();
    origin_x_ = pos.x();
    origin_y_ = pos.y();
    size_x_ = size(0);
    size_y_ = size(1);
    resolution_ = costmap_.getResolution();

  // Here we avoid doing any math to wx,wy before comparing them to
  // the bounds, so their values can go out to the max and min values
  // of double floating point.
  if (wx < origin_x_)
  {
    mx = 0;
  }
  else if (wx >= resolution_ * size_x_ + origin_x_)
  {
    mx = size_x_ - 1;
  }
  else
  {
    mx = (int)((wx - origin_x_) / resolution_);
  }

  if (wy < origin_y_)
  {
    my = 0;
  }
  else if (wy >= resolution_ * size_y_ + origin_y_)
  {
    my = size_y_ - 1;
  }
  else
  {
    my = (int)((wy - origin_y_) / resolution_);
  }
}

void GridCostmap2D::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
{
    auto pos = costmap_.getPosition();
  wx = pos.x() + (mx + 0.5) * costmap_.getResolution();
  wy = pos.y() + (my + 0.5) * costmap_.getResolution();
}

bool GridCostmap2D::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
{
    double origin_x_, origin_y_, resolution_, size_x_, size_y_;
    auto pos = costmap_.getPosition();
    auto size = costmap_.getSize();
    origin_x_ = pos.x();
    origin_y_ = pos.y();
    size_x_ = size(0);
    size_y_ = size(1);
    resolution_ = costmap_.getResolution();

  if (wx < origin_x_ || wy < origin_y_)
    return false;

  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);

  if (mx < size_x_ && my < size_y_)
    return true;

  return false;
}

void GridCostmap2D::worldToMapNoBounds(double wx, double wy, int& mx, int& my) const
{
    double origin_x_, origin_y_, resolution_;
    auto pos = costmap_.getPosition();
    origin_x_ = pos.x();
    origin_y_ = pos.y();
    resolution_ = costmap_.getResolution();
  mx = (int)((wx - origin_x_) / resolution_);
  my = (int)((wy - origin_y_) / resolution_);
}

unsigned int GridCostmap2D::cellDistance(double world_dist)
{
  double cells_dist = std::max(0.0, ceil(world_dist / getResolution()));
  return (unsigned int)cells_dist;
}

unsigned char GridCostmap2D::getCost(unsigned int mx, unsigned int my) const
{
    return (unsigned char) costmap_.atPosition("occupancy", grid_map::Position(mx, my));
}

void GridCostmap2D::setCost(unsigned int mx, unsigned int my, unsigned char cost)
{
    costmap_.atPosition("occupancy", grid_map::Position(mx, my)) = (float)cost;
}
