

## Запуск демки бекенда

## Сборка образов:
./dc.sh --profile backend_demo build

## Запуск:
./dc.sh --profile backend_demo up --rosargs world:=baylands

foxglove ждет по адресу http://localhost:8080, далее подключаемся через ws://localhost:8765

## Настройки foxglove:
- добавить панель Mower, в ее настройках выставить topic: /ui_cmd
- на панели 3D включить следующие топики:
    - /map                          - google карта
    - /exploration_polygon_marker   - рисование полигонов

## Работа
1. Рисуем полигон инструментом "Точка", замыкаем последней точкой начальную.
Полигон станет красным.
2. Нажимаем кнопку "Lets Go" на панели "Mower".
Робот должен начать двигаться по направлению к первой точке полигона.


## Запуск сегментации

1. Скачать http://kan-rt.ddns.net:18000/incoming/pidnet/, поместить файлы в папку data
2. запустить backend_demo с миром baylands

2.
./dc.sh up costmap_demo

3. В foxglove включить отображение карты /seg_costmap/costmap/costmap

4. Указать точку куда ехать через инструмент "Publish Pose".

При обнаружении дороги она будет отображена как препятствие на карте.
