./build_ws.sh

cd ../ && source devel/setup.bash

roslaunch engix_launch main.launch \
    sim:=True world:=empty robot:=turtlebot \
    control:=True config:=gazebo_turtlebot \
    global_planner:=True \
    rosbridge:=True
