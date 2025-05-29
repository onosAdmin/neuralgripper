

cd opencv_gstreamer_deepstream6.4_ros2_docker
xhost +local:docker
docker-compose up -d moveit2


docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash
cd /shared_with_docker/  && source install/setup.bash && source /opt/ros/rolling/setup.bash
ros2 launch robotic_arm7 demo.launch.py




docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash

cd /shared_with_docker/pyserial-3.5  && python3 setup.py install && cd /shared_with_docker/

cd /shared_with_docker/  &&  source install/setup.bash && source /opt/ros/rolling/setup.bash
python3 control_servos_using_esp32_moveit2.py








docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash
cd /shared_with_docker/  &&  source install/setup.bash && source /opt/ros/rolling/setup.bash
python3 ros2_arm_orchestrator.py




docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash
cd /shared_with_docker/  &&  source install/setup.bash && source /opt/ros/rolling/setup.bash
cd /shared_with_docker/  && ros2 run socket_arm_mover01 socket_arm_mover_v0.1




If you have not the  yoloros2 docker image running :
```
cd opencv_gstreamer_deepstream6.4_ros2_docker
xhost +local:docker
docker-compose run --rm yoloros2 
```


```

cd /shared_with_docker/ && python3 yolo_class_direction_provider_publisher.py

```
