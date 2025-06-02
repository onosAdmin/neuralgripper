https://moveit.picknik.ai/main/index.html
https://moveit.picknik.ai/main/doc/how_to_guides/how_to_setup_docker_containers_in_ubuntu.html
https://moveit.picknik.ai/main/doc/tutorials/quickstart_in_rviz/quickstart_in_rviz_tutorial.html


## create the docker file if it does not exist:
```
nano  docker-compose.yml
```


```
# Example command:
#   Humble on a NVIDIA system:
#   DOCKER_IMAGE=humble-humble-tutorial-source docker compose run gpu
#   Rolling without discrete graphics:
#   DOCKER_IMAGE=main-rolling-tutorial-source compose run cpu

#DOCKER_IMAGE=main-jazzy-tutorial-source compose run gpu
services:
  cpu:
    image: moveit/moveit2:main-jazzy-tutorial-source
    container_name: moveit2_container_cpu
    privileged: true
    network_mode: host
    command: /bin/bash
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - $XAUTHORITY:/root/.Xauthority
    environment:
      QT_X11_NO_MITSHM: 1
      DISPLAY: $DISPLAY
  gpu:
    image: moveit/moveit2:main-jazzy-tutorial-source
    container_name: moveit2_container_gpu
    privileged: true
    network_mode: host
    command: /bin/bash
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix
      - $XAUTHORITY:/root/.Xauthority
      - /media/data/progetti/neural_gripper/neuralgripper/shared_with_docker:/shared_with_docker
    environment:
      QT_X11_NO_MITSHM: 1
      DISPLAY: $DISPLAY
      NVIDIA_VISIBLE_DEVICES: all
      NVIDIA_DRIVER_CAPABILITIES: all
      
```



Make sure to modify:
/media/data/progetti/neural_gripper/neuralgripper/shared_with_docker:/shared_with_docker
with your folder


## Run the container

```
docker-compose run --rm gpu
```


Then open another terminal and enter in the running docker :
you can do :

docker ps 

and then

docker exec -it name_found

replacing name_found with your docker name

for example
docker exec -it movit2_gpu_run_40c5e47b63d9   /bin/bash  && source /opt/ros/rolling/setup.bash

I use this simple grep combination:

```
docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash
```



Now inside the container if you need to create a new custom arm you should run :

 ```
 cd /shared_with_docker/
 ros2 launch moveit_setup_assistant setup_assistant.launch.py
 ```
 
 
for the setup of the arm look here:

https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html
https://automaticaddison.com/complete-guide-to-the-moveit-setup-assistant-for-moveit-2/
 
Remember the arm package should be named with a "_moveit_config" at the end  or some libraries won't work correctly







If after you have configurated your custom arm you could not open the created config file, check the hidden file  .setup_assistant
you should have the correct names an paths like this:
  urdf:
    package: "robotic_arm7"
    relative_path: /shared_with_docker/moveit_arm07/robotic_arm7.urdf
  srdf:
    relative_path: /shared_with_docker/moveit_arm07/robotic_arm7/config/robotic_arm.srdf
    
check to have the correct folders (some of this folders and files will be created later from the build process) :

```
shared_with_docker/moveit_arm07/robotic_arm7$ tree
.
├── CMakeLists.txt
├── config
│   ├── initial_positions.yaml
│   ├── joint_limits.yaml
│   ├── kinematics.yaml
│   ├── moveit_controllers.yaml
│   ├── moveit.rviz
│   ├── pilz_cartesian_limits.yaml
│   ├── robotic_arm.ros2_control.xacro
│   ├── robotic_arm.srdf
│   ├── robotic_arm.urdf.xacro
│   └── ros2_controllers.yaml
├── launch
│   ├── demo.launch.py
│   ├── move_group.launch.py
│   ├── moveit_rviz.launch.py
│   ├── rsp.launch.py
│   ├── setup_assistant.launch.py
│   ├── spawn_controllers.launch.py
│   ├── static_virtual_joint_tfs.launch.py
│   └── warehouse_db.launch.py
└── package.xml

3 directories, 20 files

```




## Build the newly configured arm package using ros2 

```
cd /shared_with_docker/
colcon build --packages-select robotic_arm7
cd /shared_with_docker/ && source install/setup.bash && source /opt/ros/rolling/setup.bash


```





If you see an error saying:
The package "robotic_arm7" must not "exec_depend" on a package with the same name as this package
Open the  file : /shared_with_docker/robotic_arm7/package.xml
and remove the line where there is this:
  <exec_depend>robotic_arm7</exec_depend>



Now source the new compiled files:

```
cd /shared_with_docker/ && source install/setup.bash
```



You can run the custom arm moveit2 interface now:

``` 
ros2 launch robotic_arm7 demo.launch.py
```




If you don't see the arm:
look here:
 https://github.com/moveit/moveit2/issues/2738

Open the file  /shared_with_docker/robotic_arm7/config/joint_limits.yaml

modify alll the limits adding .0   in order to force a casting to float


Then you should also add the lines for each joint: 

 ```has_jerk_limits: false ```

And set all the  has_acceleration_limits to true:

  ``` has_acceleration_limits: true ```


 
Example of a correct file: 
 
```
# joint_limits.yaml allows the dynamics properties specified in the URDF to be overwritten or augmented as needed

# For beginners, we downscale velocity and acceleration limits.
# You can always specify higher scaling factors (<= 1.0) in your motion requests.  # Increase the values below to 1.0 to always move at maximum speed.
default_velocity_scaling_factor: 0.1
default_acceleration_scaling_factor: 0.1

# Specific joint properties can be changed with the keys [max_position, min_position, max_velocity, max_acceleration]
# Joint limits can be turned off with [has_velocity_limits, has_acceleration_limits]
joint_limits:
  joint1:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
    has_jerk_limits: false
  joint2:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
    has_jerk_limits: false
  joint3:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
    has_jerk_limits: false
  joint4:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
    has_jerk_limits: false
  rotating_base:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
    has_jerk_limits: false
```


Then you should save and compile again:

```
cd /shared_with_docker/
colcon build --packages-select robotic_arm7
source install/setup.bash

```


Now retry:

```
cd /shared_with_docker/
ros2 launch robotic_arm7 demo.launch.py
```




Now try to move the arm using the mouse





If the planning is not working and you see the error: joint must have an acceleration limits or something similar

You should modify:

/shared_with_docker/robotic_arm7/config/joint_limits.yaml 
to set the has_acceleration_limits to true for all the joints

```
  rotating_base:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
    has_jerk_limits: false




If you have this error:

``` velocity limit was defined for joint virtual_joint/x! You have to define velocity limits in the URDF or joint_limits.yaml```



modify   /shared_with_docker/robotic_arm7/config/joint_limits.yaml
Add this joint at the start:

  virtual_joint/x:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
    has_jerk_limits: false
  virtual_joint/y:
    has_velocity_limits: true
    max_velocity: 1.0
    has_acceleration_limits: true
    max_acceleration: 1.0
    has_jerk_limits: false




Remember to always compile again after a change!!!






If you have this  error:
 - XacroException: No such file or directory: /shared_with_docker/install/robotic_arm7/share/robotic_arm7//shared_with_docker/moveit_arm07/robotic_arm7.urdf [Errno 2] No such file or directory: '/shared_with_docker/install/robotic_arm7/share/robotic_arm7//shared_with_docker/moveit_arm07/robotic_arm7.urdf'
 - InvalidFrontendLaunchFileError: The launch file may have a syntax error, or its format is unknown


You should look at 
config/robotic_arm.urdf.xacro

I modified this:
    <xacro:include filename="$(find robotic_arm7)//shared_with_docker/moveit_arm07/robotic_arm7.urdf" />

in  this
    <xacro:include filename="/shared_with_docker/moveit_arm07/robotic_arm7.urdf" />



then rebuild with:
colcon build --packages-select robotic_arm7  && source install/setup.bash






While on a terminal you have running ros2 launch robotic_arm7 demo.launch.py

Open a second terminal  inside the docker:
```
docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash

```



if you want to controll the robot arm using serial port , first install pyserial

Since I put pyserial ready to install folder in this repository you could just do:

```
cd /shared_with_docker/pyserial-3.5  && python3 setup.py install  && cd ..

```

Now run the python script:

```
cd /shared_with_docker/ && source /opt/ros/rolling/setup.bash  && source install/setup.bash

python3 control_servos_from_manual_moveit2.py


```



Now open a third terminal inside docker :

```
docker exec -it $(docker ps | grep moveit2 | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash

```

You can run the python controll script, this script will ask moveit2 to move the robotic arm to different coordinates
control_servos_from_manual_moveit2.py  will read the current arm joints positions convert radiant to degrees and
 send each value to the esp32 that will then controll each servomotor.


```

cd /shared_with_docker/ && source /opt/ros/rolling/setup.bash  && source install/setup.bash

python3 planning_move_test.py

```


Make sure to have the esp32 flashed with the correct circuitpython code connected to the usb of your pc before starting the docker
otherwise docker will not have access tro the serial port and the script will not work.




Some usefull links:

https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html

https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html
https://github.com/moveit/moveit2_tutorials/blob/main/doc/examples/setup_assistant/setup_assistant_tutorial.rst

tutorial of 3d printed arm here
https://github.com/AIWintermuteAI/ros-moveit-arm/tree/master/my_arm_xacro/config
https://www.instructables.com/ROS-MoveIt-Robotic-Arm/


https://medium.com/@santoshbalaji/manipulation-with-moveit2-visualizing-robot-arm-in-simulation-1-8cd3a46d42b4



https://github.com/ut-ims-robotics/pool-thesis-2023-moveit2-examples/tree/main/python_examples



moveit2 test in c++
https://automaticaddison.com/how-to-create-your-first-c-moveit-2-project/


moveit2 object pickup:
https://moveit.picknik.ai/humble/doc/examples/move_group_interface/move_group_interface_tutorial.html

latest_test:
https://www.youtube.com/watch?v=c6Bxbq8UdaI
https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html
