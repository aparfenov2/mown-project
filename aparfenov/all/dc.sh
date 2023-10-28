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

VOLUMES=()
for f in $(find ws/src -type l); do
    if ! readlink -f $f > /dev/null; then
        echo "failed to map $f"
    fi
    VOLUMES+=("$(readlink -f $f):/cdir/$f")
done

[ -e "data" ] && {
    VOLUMES+=("$(readlink -f data):/cdir/data")
}
DCG="docker-compose.yaml"
echo "version: \"3.9\"" > $DCG
echo "x-common-mounts: &common-mounts" >> $DCG
for f in "${VOLUMES[@]}"; do
    echo "    - $f" >> $DCG
done

cat docker-compose.main.yaml >> $DCG
xhost +
set -ex
ROSARGS="${ROSARGS[@]}" docker compose ${DCARGS[@]}
