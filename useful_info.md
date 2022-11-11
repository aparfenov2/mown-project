create package in current dir
    ./run_cmd.sh catkin_create_pkg simple_layers roscpp costmap_2d dynamic_reconfigure 

view segmentation output
    ./run_cmd.sh rosrun image_view image_view image:=/segmentation_node/mask_color
