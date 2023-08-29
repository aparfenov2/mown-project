packages='engix_launch '
packages+="engix_gazebo "
packages+="global_planner "
packages+="move_controller "
packages+="velodyne_description "
packages+="tb_gazebo_description "
packages+="tb_gazebo "
packages+="tb_gazebo_msgs "
packages+="tb_gazebo_plugins "
packages+="gazebo_bridge_localization "
packages+="engix_msgs "
packages+="engix_utils "

source /opt/ros/noetic/setup.bash
rosdep update
cd ../ && catkin init && catkin build $packages