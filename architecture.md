
# Obstacle avoidance

Data sources: lidar, segmented pointcloud
see all.launch

1. Segmented pointcloud generation
- in simulator
    Camera -> img  -> ddrnet -> segmetation mask -> + -> segmented pcl
    Camera -> depth map                          ->

- in reality
    ZED -> segmented pcl

2. Pointclouds projection
lidar -> pcl  -> map_server (projection to world coords, raytracing to "occupancy" layer )
segmented pcl -> map_server (projection to world coords, direct projection to "color" layer )

2a. map publication
    map_server -> conv2occgrid -> nav_msgs/OccupancyGrid
    map_server -> grid_map_msgs::GridMap (occupancy + color layers)

3. planning
    point planner - uses nav_msgs/OccupancyGrid
    coverage planner - uses nav_msgs/OccupancyGrid
    trajectory rollout planner - uses grid_map_msgs::GridMap
        - input: point to go tu
        - input: route from coverage planner
        - output: trajectory
