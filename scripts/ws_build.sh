
function build_all() {
    cd /catkin_ws
    catkin build $(catkin list -u -d src/)
}

function build_folder() {
    catkin build $(catkin list -u -d .)
}

function build_this() {
    catkin build --this
}

function main() {
    echo "$@"
    # if [[ $# -eq 1]]
    # then
    #     build_all
    #     ws_source.sh
    #     exit 1
    # fi
    if [ "$#" -eq 0 ]; then
        echo "Build all project" 
        build_all
    elif [ "$#" -eq 1 ]; then
        case $1 in
        --all)
            echo "Build all project"
            build_all
            ;;

        --folder)
            echo "Build all packages in a given directory"
            build_folder
            ;;

        --this)
            echo "Build the package containing the current working directory"
            build_this
            ;;

        *)
            echo "Wrong argument. Usage: ws_build [--all | --folder | --this]"
            ;;
        esac
    else
        echo "Wrong argument number"
        exit 0
    fi

    ws_source.sh
}

main "$@"