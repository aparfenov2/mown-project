# Запуск артифактов через docker-compose

## Настройка свежего компа
- required: Ubuntu >=18
- install docker, docker compose,
    https://docs.docker.com/engine/install/ubuntu/
- setup docker without sudo
    https://docs.docker.com/engine/install/linux-postinstall/
- apt-get install nvidia-container-runtime
    https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
- Прописать в /etc/docker/daemon.json
```
{
   "insecure-registries" : ["kan-rt.ddns.net:8929"]
}
```
- Залогиниться в мой сервер чтобы подтянуть образы
```
docker login kan-rt.ddns.net:8929
    user: docker
    pwd: Engix_1835
```

- установить git lfs
curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
sudo apt-get install git-lfs

- Сгенерить и добавить ssh ключ в аккаунт github чтобы склонировать репу с субмодулями
```
ssh-keygen, add pub to repo
git clone git://mower-project
git submodule update --init --recursive
```

## Подготовка
cd artifacts

- подтянуть образы
```
make pull
```
- или собрать образы
```
make build_images
```
- скомпилить воркспейс
```
make build
```

## Запуск
- запустить всё вместе с нейронкой
```
make all
```
- или в режиме демона
```
DAEMON="-d" make all
```
- всё, но без нейронки
```
make backend_demo
```

- foxglove ждет по адресу http://localhost:8080, далее подключаемся через ws://localhost:8765

## Настройки foxglove
- добавить панель Mower, в ее настройках выставить topic: /ui_cmd
- на панели 3D включить следующие топики:
    - /map                          - google карта
    - /exploration_polygon_marker   - рисование полигонов

## Работа
1. Рисуем полигон инструментом "Точка", замыкаем последней точкой начальную.
Полигон станет красным.
2. Нажимаем кнопку "Lets Go" на панели "Mower".
    - Робот должен начать двигаться по направлению к первой точке полигона.
    - сейчас не работает, т.к. нужен полноценный планировщик

3. ИЛИ Указать точку куда ехать через инструмент "Publish Pose".
    - Робот должен начать двигаться по направлению к указанной точке

- Для просмотра карты сегментации в foxglove включить отображение карты /seg_costmap/costmap/costmap
    - При обнаружении дороги она будет отображена как препятствие на карте.


## Проблемы, FAQ
3rd_party/gazebo_models не содержит мир baylands
git submodule update --init --recursive
cd 3rd_party/gazebo_models
git fetch --all
git reset --hard origin/master
