
function run_planning() {
    source /catkin_ws/devel/setup.bash
    roslaunch navigation_launch main.launch global_planner:=True config:=mower_v1
}

function run_control() {
    source /catkin_ws/devel/setup.bash
    roslaunch navigation_launch main.launch control:=True config:=mower_v1
}

function run_localization() {
    source /catkin_ws/devel/setup.bash
    roslaunch navigation_launch main.launch xavier_localization:=True
}

function run_navigation() {
    source /catkin_ws/devel/setup.bash
    roslaunch navigation_launch main.launch control:=True \
        global_planner:=True \
        xavier_localization:=True \
        config:=mower_v1
}

function main() {
    if [ "$#" -eq 1 ]; then
        case $1 in
        --control)
            echo "Run control"
            run_control
            ;;

        --planning)
            echo "Run planning"
            run_planning
            ;;

        --localization)
            echo "Run localization"
            run_localization
            ;;

        --navigation)
            echo "Run navigation"
            run_navigation
            ;;

        *)
            echo "Wrong argument. Usage: ws_build [--control | --planning | --localization | -- navigation]"
            ;;
        esac
    else
        echo "Wrong argument number"
        exit 0
    fi
}

main "$@"