Эксперимент: новый плагин по типу inflation_layer для обновления слоя costmap2d на основе данных сегментации.

http://wiki.ros.org/costmap_2d/Tutorials/Creating%20a%20New%20Layer

Как воспроизвести:
./run_cmd.sh --build
./run_all.sh --sim --segm --segm_net world:=baylands
./run_all.sh mb:=true
./run_all.sh --rviz
