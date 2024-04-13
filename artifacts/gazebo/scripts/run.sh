cd /catkin_ws

source /catkin_ws/devel/setup.bash

rosdep update

WORLD="empty"
ROBOT="turtlebot"
USE_GAZEBO_SIM_TIME="True"
GUI="True"


roslaunch simulator_launch gazebo.launch world:=$WORLD robot:=$ROBOT use_sim_time_in_gazebo:=$USE_GAZEBO_SIM_TIME gui:=$GUI