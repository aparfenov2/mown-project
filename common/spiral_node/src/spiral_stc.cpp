//
// Copyright [2020] Nobleo Technology"  [legal/copyright]
//
#include <algorithm>
#include <iostream>
#include <list>
#include <string>
#include <vector>

#include "full_coverage_path_planner/spiral_stc.h"
#include <pluginlib/class_list_macros.h>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(full_coverage_path_planner::SpiralSTC, nav_core::BaseGlobalPlanner)

namespace full_coverage_path_planner
{
void SpiralSTC::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!initialized_)
  {
    // Create a publisher to visualize the plan
    ros::NodeHandle private_nh("~/");
    ros::NodeHandle nh, private_named_nh("~/" + name);

    plan_pub_ = private_named_nh.advertise<nav_msgs::Path>("plan", 1);
    // Try to request the cpp-grid from the cpp_grid map_server
    // cpp_grid_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");

    // Define  robot radius (radius) parameter
    float robot_radius_default = 0.5f;
    private_named_nh.param<float>("robot_radius", robot_radius_, robot_radius_default);
    // Define  tool radius (radius) parameter
    float tool_radius_default = 0.5f;
    private_named_nh.param<float>("tool_radius", tool_radius_, tool_radius_default);
    std::string default_frame_id = "map";
    private_named_nh.param<std::string>("frame_id", frame_id_, default_frame_id);
    initialized_ = true;
    costmap_ros_ = costmap_ros;

    std::string coverage_map_topic;
    std::string default_coverage_map_topic = "/coverage_grid";
    private_named_nh.param<std::string>("coverage_map_topic", coverage_map_topic, default_coverage_map_topic);

    map_sub_ = nh.subscribe(coverage_map_topic, 1, &SpiralSTC::incomingCoverageMap, this);
  }
}

void SpiralSTC::incomingCoverageMap(const nav_msgs::OccupancyGridConstPtr& map) {
    geometry_msgs::PoseStamped start;
    Point_t startPoint;
    ROS_INFO_THROTTLE(15, "incomingCoverageMap: recieved update");
    coverage_grid_.empty();
  if (!parseGrid(*map, coverage_grid_, robot_radius_ * 2, tool_radius_ * 2, start, startPoint ))
  {
    ROS_ERROR("incomingCoverageMap: Could not parse retrieved grid");
  }

}

std::list<gridNode_t> SpiralSTC::spiral(std::vector<std::vector<bool> > const& grid, std::list<gridNode_t>& init,
                                        std::vector<std::vector<bool> >& visited)
{
  int dx, dy, dx_prev, x2, y2, i, nRows = grid.size(), nCols = grid[0].size();
  // Spiral filling of the open space
  // Copy incoming list to 'end'
  std::list<gridNode_t> pathNodes(init);
  // Create iterator for gridNode_t list and let it point to the last element of end
  std::list<gridNode_t>::iterator it = --(pathNodes.end());
  if (pathNodes.size() > 1)  // if list is length 1, keep iterator at end
    it--;                    // Let iterator point to second to last element

  gridNode_t prev = *(it);

  // escape from coverage spot
  // trace the spot in 4 directions until first uncovered open cell or map boundary
  // select the shortest path
  std::list<std::list<gridNode_t>> coverage_spot_exits;
  // Initialize spiral direction towards y-axis
  dx = 0;
  dy = 1;
  for (int i = 0; i < 4; ++i)
  {
    x2 = pathNodes.back().pos.x + dx;
    y2 = pathNodes.back().pos.y + dy;
    std::list<gridNode_t> exit_path;

    while (
      x2 >= 0 && x2 < nCols && y2 >= 0 && y2 < nRows &&
      grid[y2][x2] == eNodeOpen && visited[y2][x2] == eNodeOpen &&
      coverage_grid_[y2][x2] == eNodeClosed
    )
    {
      Point_t new_point = { x2, y2 };
      gridNode_t new_node =
      {
        new_point,  // Point: x,y
        0,          // Cost
        0,          // Heuristic
      };

      exit_path.push_back(new_node);
      dx2 = dx2 + dx;
      dy2 = dy2 + dy;
    }
    coverage_spot_exits.push_back(exit_path);

    dx_prev = dx;
    dx = -dy;
    dy = dx_prev;
  }
  std::list<gridNode_t> shortest_exit_path;
  for (int i = 0; i < 4; ++i)
  {
    if (coverage_spot_exits[i].size() < shortest_exit_path.size())
    {
      shortest_exit_path = coverage_spot_exits[i];
    }
  }
  pathNodes.insert(pathNodes.end(), shortest_exit_path.begin(), shortest_exit_path.end());

  // unwinding Spiral
  bool done = false;
  while (!done)
  {
    if (it != pathNodes.begin())
    {
      // turn ccw
      dx = pathNodes.back().pos.x - prev.pos.x;
      dy = pathNodes.back().pos.y - prev.pos.y;

      dx_prev = dx;
      dx = -dy;
      dy = dx_prev;
    }
    else
    {
      // Initialize spiral direction towards y-axis
      dx = 0;
      dy = 1;
    }
    done = true;

    // try next step
    // list open grid cells as open_grid_list
    // list all open that not covered as open_grid_not_covered
    //      if any open_grid_not_covered - go there
    //          mark (open open_grid_list - open_grid_not_covered) as visited
    //      if none open_grid_not_covered - go first open open_grid_list
    //          mark all open_grid_list as visited

    std::vector<gridNode_t> open_grid_list;

    int dx2 = dx, dy2 = dy;

    for (int i = 0; i < 4; ++i)
    {
      x2 = pathNodes.back().pos.x + dx2;
      y2 = pathNodes.back().pos.y + dy2;
      if (x2 >= 0 && x2 < nCols && y2 >= 0 && y2 < nRows)
      {
        if (grid[y2][x2] == eNodeOpen && visited[y2][x2] == eNodeOpen)
        {
          Point_t new_point = { x2, y2 };
          gridNode_t new_node =
          {
            new_point,  // Point: x,y
            coverage_grid_[y2][x2] ? 1 : 0,          // Cost
            0,          // Heuristic
          };
            open_grid_list.push_back(new_node);
        }
      }
      // try next direction cw
      dx_prev = dx2;
      dx2 = dy2;
      dy2 = -dx_prev;
    }

    int first_open_idx = -1;
    int first_open_uncovered_idx = -1;

    ROS_ASSERT(open_grid_list.size() <= 4);

    for (int i=0; i < open_grid_list.size(); i++) {
        auto pt = open_grid_list[i];
        if (first_open_idx < 0) {
            first_open_idx = i;
        }
        if (first_open_uncovered_idx < 0) {
            if (!pt.cost) {
                first_open_uncovered_idx = i;
            }
        }
    }

    int first_possibly_uncovered_idx = first_open_uncovered_idx >= 0 ? first_open_uncovered_idx : first_open_idx;

    if (first_open_idx >= 0) {
        auto new_node = open_grid_list[first_open_idx];
        // ROS_INFO("%d %d first_open_uncovered_idx %d first_open_idx %d", new_node.pos.y, new_node.pos.x, first_open_uncovered_idx, first_open_idx);
          prev = pathNodes.back();
          new_node.cost = 0;
          pathNodes.push_back(new_node);
          it = --(pathNodes.end());
          visited[new_node.pos.y][new_node.pos.x] = eNodeVisited;  // Close node
          done = false;
    }
  }
  return pathNodes;
}

std::list<Point_t> SpiralSTC::spiral_stc(std::vector<std::vector<bool> > const& grid,
                                          Point_t& init,
                                          int &multiple_pass_counter,
                                          int &visited_counter)
{
  int x, y, nRows = grid.size(), nCols = grid[0].size();
  // Initial node is initially set as visited so it does not count
  multiple_pass_counter = 0;
  visited_counter = 0;

  std::vector<std::vector<bool> > visited;
  visited = grid;  // Copy grid matrix
  x = init.x;
  y = init.y;

  Point_t new_point = { x, y };
  gridNode_t new_node =
  {
    new_point,  // Point: x,y
    0,          // Cost
    0,          // Heuristic
  };
  std::list<gridNode_t> pathNodes;
  std::list<Point_t> fullPath;
  pathNodes.push_back(new_node);
  visited[y][x] = eNodeVisited;

#ifdef DEBUG_PLOT
  ROS_INFO("Grid before walking is: ");
  printGrid(grid, visited, fullPath);
#endif

  pathNodes = SpiralSTC::spiral(grid, pathNodes, visited);                // First spiral fill
  std::list<Point_t> goals = map_2_goals(visited, eNodeOpen);  // Retrieve remaining goalpoints
  // Add points to full path
  std::list<gridNode_t>::iterator it;
  for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
  {
    Point_t newPoint = { it->pos.x, it->pos.y };
    visited_counter++;
    fullPath.push_back(newPoint);
  }
  // Remove all elements from pathNodes list except last element
  pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));      // dwa planner cannot plan outside rolling map

#ifdef DEBUG_PLOT
  ROS_INFO("Current grid after first spiral is");
  printGrid(grid, visited, fullPath);
  ROS_INFO("There are %d goals remaining", goals.size());
#endif
  while (goals.size() != 0)
  {
    // Remove all elements from pathNodes list except last element.
    // The last point is the starting point for a new search and A* extends the path from there on
    pathNodes.erase(pathNodes.begin(), --(pathNodes.end()));    // dwa planner cannot plan outside rolling map
    visited_counter--;  // First point is already counted as visited
    // Plan to closest open Node using A*
    // `goals` is essentially the map, so we use `goals` to determine the distance from the end of a potential path
    //    to the nearest free space
    bool resign = a_star_to_open_space(grid, pathNodes.back(), 1, visited, goals, pathNodes);
    if (resign)
    {
#ifdef DEBUG_PLOT
      ROS_INFO("A_star_to_open_space is resigning", goals.size());
#endif
      break;
    }

    // Update visited grid
    for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
    {
      if (visited[it->pos.y][it->pos.x])
      {
        multiple_pass_counter++;
      }
      visited[it->pos.y][it->pos.x] = eNodeVisited;
    }
    if (pathNodes.size() > 0)
    {
      multiple_pass_counter--;  // First point is already counted as visited
    }

#ifdef DEBUG_PLOT
    ROS_INFO("Grid with path marked as visited is:");
    gridNode_t SpiralStart = pathNodes.back();
    printGrid(grid, visited, pathNodes, pathNodes.front(), pathNodes.back());
#endif

    // Spiral fill from current position
    pathNodes = spiral(grid, pathNodes, visited);

#ifdef DEBUG_PLOT
    ROS_INFO("Visited grid updated after spiral:");
    printGrid(grid, visited, pathNodes, SpiralStart, pathNodes.back());
#endif

    goals = map_2_goals(visited, eNodeOpen);  // Retrieve remaining goalpoints

    for (it = pathNodes.begin(); it != pathNodes.end(); ++it)
    {
      Point_t newPoint = { it->pos.x, it->pos.y };
      visited_counter++;
      fullPath.push_back(newPoint);
    }
  }

  return fullPath;
}

bool SpiralSTC::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                         std::vector<geometry_msgs::PoseStamped>& plan)
{
  if (!initialized_)
  {
    ROS_ERROR("This planner has not been initialized yet, but it is being used, please call initialize() before use");
    return false;
  }
  else
  {
    ROS_INFO("Initialized!");
  }

  clock_t begin = clock();
  Point_t startPoint;

  /********************** Get grid from server **********************/
  std::vector<std::vector<bool> > grid;
  // nav_msgs::GetMap grid_req_srv;
  // ROS_INFO("Requesting grid!!");
  // if (!cpp_grid_client_.call(grid_req_srv))
  // {
  //   ROS_ERROR("Could not retrieve grid from map_server");
  //   return false;
  // }

  if (!parseGrid(costmap_ros_->getCostmap(), grid, robot_radius_ * 2, tool_radius_ * 2, start, startPoint))
  {
    ROS_ERROR("Could not parse retrieved grid");
    return false;
  }

#ifdef DEBUG_PLOT
  ROS_INFO("Start grid is:");
  std::list<Point_t> printPath;
  printPath.push_back(startPoint);
  printGrid(grid, grid, printPath);
#endif

  std::list<Point_t> goalPoints = spiral_stc(grid,
                                              startPoint,
                                              spiral_cpp_metrics_.multiple_pass_counter,
                                              spiral_cpp_metrics_.visited_counter);
  ROS_INFO("naive cpp completed!");
  ROS_INFO("Converting path to plan");

  parsePointlist2Plan(start, goalPoints, plan);
  // Print some metrics:
  spiral_cpp_metrics_.accessible_counter = spiral_cpp_metrics_.visited_counter
                                            - spiral_cpp_metrics_.multiple_pass_counter;
  spiral_cpp_metrics_.total_area_covered = (4.0 * tool_radius_ * tool_radius_) * spiral_cpp_metrics_.accessible_counter;
  ROS_INFO("Total visited: %d", spiral_cpp_metrics_.visited_counter);
  ROS_INFO("Total re-visited: %d", spiral_cpp_metrics_.multiple_pass_counter);
  ROS_INFO("Total accessible cells: %d", spiral_cpp_metrics_.accessible_counter);
  ROS_INFO("Total accessible area: %f", spiral_cpp_metrics_.total_area_covered);

  // TODO(CesarLopez): Check if global path should be calculated repetitively or just kept
  // (also controlled by planner_frequency parameter in move_base namespace)

  ROS_INFO("Publishing plan!");
  publishPlan(plan);
  ROS_INFO("Plan published!");
  ROS_DEBUG("Plan published");

  clock_t end = clock();
  double elapsed_secs = static_cast<double>(end - begin) / CLOCKS_PER_SEC;
  std::cout << "elapsed time: " << elapsed_secs << "\n";

  return true;
}
}  // namespace full_coverage_path_planner
