
function run_sim() {
    . ws_source.sh
    roslaunch my_utils_common all.launch rviz:=false sim:=true rosbridge:=true robot:=turtlebot
}

function run_planning() {
    . ws_source.sh
    roslaunch my_utils_common all.launch planning:=true robot:=turtlebot
}

function run_gui() {
    . ws_source.sh
    # cd /catkin_ws/src/gui/command_panel
    # python3 main.py
    roslaunch command_panel run.launch
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