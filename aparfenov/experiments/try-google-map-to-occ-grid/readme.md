эксперимент по запуску гугл карты в foxglove

./run_all.sh --build
./run_all.sh --sim            # симулятор
./run_all.sh backend:=true    # нода гугл-карт. Запускает Firefox в фоне!
./run_all.sh --pubfix         # публикует координаты через rostopic pub
./run_all.sh --rosbridge      # запускает foxglove_bridge
./run_all.sh --studio         # локальный сервер foxglove, из браузера заходить в http://localhost:8080

В foxglove добавить 3D панель, в ее свойствах выбрать топик /map, режим отображения RGBA.

