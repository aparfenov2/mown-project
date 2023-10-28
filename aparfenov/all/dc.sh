
[ -z "$1" ] && {
    echo specify up [service_name] or down [something]
    exit 1
}

VOLUMES=()
for f in $(find ws/src -type l); do
    VOLUMES+=("$(readlink -f $f):/cdir/$f")
done

[ -e "data" ] && {
    VOLUMES+=("$(readlink -f data):/cdir/data")
}
DCG="_dc-generated.yaml"
echo "version: \"3.9\"" > $DCG
echo "x-common-mounts: &common-mounts" >> $DCG
echo "    - ./.ros:/root/.ros" >> $DCG
echo "    - /dev:/dev" >> $DCG
echo "    - ./:/cdir" >> $DCG
echo "    - /home/.gazebo/models:/root/.gazebo/models" >> $DCG

for f in "${VOLUMES[@]}"; do
    echo "    - $f" >> $DCG
done

cat docker-compose.yaml >> $DCG

DOCKER_COMPOSE="docker-compose -f $DCG"

[ "$1" == "up" ] && {
    shift
    ${DOCKER_COMPOSE} up --no-recreate $@
    exit $?
}
[ "$1" == "down" ] && {
    shift
    ${DOCKER_COMPOSE} down --remove-orphans $@
    exit $?
}
[ "$1" == "run" ] && {
    shift
    ${DOCKER_COMPOSE} run --rm $@
    exit $?
}
${DOCKER_COMPOSE} $@
