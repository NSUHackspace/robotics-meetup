# Turtlebot 3



```bash
ros2 pkg list | grep turtlebot3
```

```bash
sudo apt install ros-$ROS_DISTRO-turtlebot3*
```

```bash
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:`ros2 pkg \
prefix turtlebot3_gazebo \
`/share/turtlebot3_gazebo/models/

ros2 launch turtlebot3_gazebo empty_world.launch.py
```


```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

## Дополнительная информация

1. [Установка tutrtlebot3 для ROS2 + Gazebo classic от ROS Industrial training](
https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Turtlebot.html)



# cartographer_ros

[Cartographer_ros](https://google-cartographer-ros.readthedocs.io/en/latest/)

# cartographer_slam

a) Создаем новый пакет cartographer_slam в подкаталоге src рабочего каталога ros2_ws.

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python cartographer_slam
```

b) Дополнительно создаем подкаталоги launch & config внутри пакета ros2_ws/src/cartographer_slam.

c) Создаем launch-файл для пакета cartographer.launch.py, который запустит 2 ноды.

```bash
cd ~/ros2_ws
colcon build --packages-select cartographer_slam
source ~/ros2_ws/install/setup.bash
```

```bash
ros2 launch cartographer_slam cartographer.launch.py

```

модифицируем setup.py
```py
import os
from glob import glob
...
# добавить после  ('share/' + package_name, ['package.xml']),
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
(os.path.join('share', package_name, 'config'), glob('config/*')),
```

rviz2
+map, laser scan -> всё сохранить в конфиг rviz'a

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

## Сохранить карту 

```bash
ros2 run nav2_map_server map_saver_cli -f turtlebot_area
```


# map_server

```bash
ros2 pkg create --build-type ament_python map_server
```

в директории пакета map_server создаем дополнительно каталоги launch & config. Копируем карту из cartographer_slam/config в map_server/config

Создаем launch-файл nav2_map_server.launch.py

Запускаем rviz2
```bash
rviz2
```

Запускаем launch-файл
```bash
source install/setup.bash
ros2 launch map_server nav2_map_server.launch.py
```


если карта не отображается - проверяем QoS

# AMCL

Локализация робота происходит в момент, когда публикуется преобразование между системами координа  /map и /odom.


Параметры rviz2 можно уточнить с помощью 
```bash
ros2 topic info /topic_name --verbose
```

Например, у топика /map Durability policy == Transient Local


## About robot localization
```bash
rviz2 -d src/localization_server/config/localizer_rviz_config.rviz 
```



```bash
[INFO] [1698065544.532446801] [rviz2]: Setting estimate pose: Frame:map, Position(-0.181196, 0.0297273, 0), Orientation(0, 0, -0.723032, 0.690815) = Angle: -1.61636

[INFO] [1698066570.081279584] [rviz2]: Setting estimate pose: Frame:map, Position(-0.253993, -0.0782617, 0), Orientation(0, 0, -0.706052, 0.70816) = Angle: -1.56782
```


```bash
ros2 service call /reinitialize_global_localization std_srvs/srv/Empty
```

# Path planning server

```bash
ros2 pkg create path_planning_server
```

создать launch & config в path_planning_server

создать pathplanner.launch.py


### Примечание 
This PR 2867 renames the nav2_recoveries to nav2_behaviors.

In navigation_launch.py recoveries_server -> behavior_server and nav2_recoveries -> nav2_behaviors. In nav2_params.yaml recovery_plugins -> behavior_plugins and nav2_recoveries -> nav2_behaviors.

И че-т он потом не запускается...
