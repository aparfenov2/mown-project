WEBVIZ_DOCKER_NAME=webviz

if [[ "$(docker images -q cruise/webviz 2> /dev/null)" == "" ]]; then
    docker run -p 8080:8080 --name $WEBVIZ_DOCKER_NAME cruise/webviz
else
    docker start WEBVIZ_DOCKER_NAME
fi