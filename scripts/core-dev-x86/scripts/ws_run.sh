
function run_sim() {
    . ws_source.sh
    roslaunch engix_launch main.launch sim:=True robot:=turtlebot world:=empty
}

function run_planning() {
    . ws_source.sh
    roslaunch my_utils_common all.launch planning:=true robot:=ya_model
}

function run_gui() {
    . ws_source.sh
    # cd /catkin_ws/src/gui/command_panel
    # python3 main.py
    roslaunch command_panel run.launch
}

function run_rosbridge() {
    . ws_source.sh
    # cd /catkin_ws/src/gui/command_panel
    # python3 main.py
    roslaunch engix_launch main.launch rosbridge:=True
}

function run_navigation() {
    . ws_source.sh
    roslaunch engix_launch main.launch control:=True global_planner:=True config:=gazebo_turtlebot
}

function run_foxglove_bridge() {
    . ws_source.sh
    roslaunch engix_launch main.launch foxglove_bridge:=True
}

function main() {
    if [ "$#" -eq 1 ]; then
        case $1 in
        --sim)
            echo "Run simulator"
            run_sim
            ;;

        --planning)
            echo "Run planning"
            run_planning
            ;;

        --navigation)
            echo "Run navigation"
            run_navigation
            ;;

        --foxglove-bridge)
            echo "Run foxglove_bridge"
            run_foxglove_bridge
            ;;

        --rosbridge)
            echo "Run rosbridge"
            run_rosbridge
            ;;

        --gui)
            echo "Run command panel"
            run_gui
            ;;

        *)
            echo "Wrong argument. Usage: ws_build [--sim | --planning | --gui]"
            ;;
        esac
    else
        echo "Wrong argument number"
        exit 0
    fi
}

main "$@"