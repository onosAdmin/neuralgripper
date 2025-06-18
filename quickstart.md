


Terminal1:
 
cd opencv_gstreamer_deepstream6.4_ros2_docker
xhost +local:docker
docker-compose up -d moveit2


docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash
cd /shared_with_docker/ && source /opt/ros/rolling/setup.bash && source install/setup.bash 
cd /shared_with_docker/  && colcon build --packages-select robotic_arm7  && source install/setup.bash
ros2 launch robotic_arm7 demo.launch.py




Terminal2:

docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash

cd /shared_with_docker/pyserial-3.5  && python3 setup.py install && cd /shared_with_docker/

cd /shared_with_docker/  &&  source install/setup.bash && source /opt/ros/rolling/setup.bash
python3 control_servos_using_esp32_moveit2.py







Terminal3:
docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash
cd /shared_with_docker/  &&  source /opt/ros/rolling/setup.bash
cd /shared_with_docker/  && colcon build --packages-select socket_arm_mover01
cd /shared_with_docker/  &&  source install/setup.bash && source /opt/ros/rolling/setup.bash
cd /shared_with_docker/  && ros2 run socket_arm_mover01 socket_arm_mover_v0.1










Terminal4:

docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash
cd /shared_with_docker/  &&  source install/setup.bash && source /opt/ros/rolling/setup.bash
python3 ros2_arm_orchestrator.py



Terminal5:

```
cd opencv_gstreamer_deepstream6.4_ros2_docker
xhost +local:docker
docker-compose run --rm yoloros2 

cd /shared_with_docker/ && python3 yolo_class_direction_provider_publisher.py

```





If you want people follower, disable all the moveit2 running scripts and then on terminal5:

```
cd /shared_with_docker/people_follower  && python3  people_follower01.py
```





If you want yolo pose command, disable all the moveit2 running scripts and then on terminal5:
cd /shared_with_docker/pyserial-3.5  && python3 setup.py install && cd /shared_with_docker/
cd /shared_with_docker/yolo_pose_arm_controller/
python3 yolo_pose_arm_controller.py 








apt-get update
 apt-get install python3-pip
python3 -m pip install torch==2.6.0     torchvision==0.21.0     ultralytics==8.3.111     numpy
apt install python3.12-venv
python3 -m venv .env
 source .env/bin/activate
