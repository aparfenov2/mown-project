# docker build -t $(cat image) .
docker build --platform arm -t $(cat image) .
