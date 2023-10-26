python gen_docker_compose.py

[ -z "$1" ] && {
    echo specify up [service_name] or down [something]
    exit 1
}

[ "$1" == "up" ] && {
    shift
    docker-compose up --no-recreate $@
    exit $?
}
[ "$1" == "down" ] && {
    shift
    docker-compose down --remove-orphans $@
    exit $?
}
docker-compose $@
