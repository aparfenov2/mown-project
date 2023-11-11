set -e

DCARGS=()
ROSARGS=()
STATE="dc_cmds"

while [[ "$#" -gt 0 ]]; do

    [ "${STATE}" == "ros_cmds" ] && {
        ROSARGS+=("$1")
        shift
        continue
    }

    [ "${STATE}" == "dc_cmds" ] && {
        [ "$1" == "up" ] && {
            DCARGS+=("$1")
            DCARGS+=("--no-recreate")
            shift
            continue
        }
        [ "$1" == "down" ] && {
            DCARGS+=("$1")
            DCARGS+=("--remove-orphans")
            shift
            continue
        }
        [ "$1" == "--rosargs" ] && {
            STATE="ros_cmds"
            shift
            continue
        }
        DCARGS+=("$1")
        shift
        continue
    }
done

call gen_docker_compose.sh
set -ex
ROSARGS="${ROSARGS[@]}" docker compose ${DCARGS[@]}
