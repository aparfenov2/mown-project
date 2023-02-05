# benewake_ce30c_ros_melodic
![Image alt](https://github.com/Yarulik/benewake_ce30c_ros_melodic/blob/main/CE30c.PNG)

 ## Инструкция по запуску лидара на ROS Melodic
На других ROS не тестировал, но на Raspbian Buster пример из CE30_sdk_test, работал

## Установка
>> git clone https://github.com/Yarulik/benewake_ce30c_ros_melodic.git

папку benewake_ce30c поместить или скопировать в "рабочую директория" ROS (например в catkin_ws/src)


#### в ~/.bashrc должно быть прописан localhost, MASTER URI


#### Если при catkin_make выйдет ошибка драйвера:
>> /usr/bin/ld: skipping incompatible /home/nvidia/Загрузки/SDK/CE30-C_ROS/libbw_ce30v2.0.so when searching for -lbw_ce30v2.0

#### то нужно на откомпилировать этот файл, перейдя в директорию 
>> cd ~/benewake_ce30c_ros_melodic/benewake_ce30c_sdk_linux/sources и сделать make 
После чего перенести или заменить файл libbw_ce30v2.0.so на новый в catkin_ws/src/benewake_c30c 

#### теперь пробуем снова 
>> catlin_make


#### Возможно потребуется установить 
>> sensor_msgs, std_msgs и д.р., об этом сообщит компилятор

##### По умолчанию адресс лидара 192.168.1.80
Заходим в настройку сетевой карты и прописываем DHCP вручную, ipv4: 192.168.1.2 (ниже 192.168.1.80) маска 255.255.255.0, сохраняем и подключаем лидар.

