# neuralgripper
A neural network controlled robotic arm gripper






## Intro
This project is built with 2 docker images running at the same time and talking to each other, 
one docker image (master_image) is in charge of yolo vision detecting the object to pickup and passing the command to the other docker image (slave_image) that is running moveit2
a python program runnig on slave_image will receive the message from master_image and will then ask moveit2 to plan the robot arm joint rotations for the new position.
Once calculated the result joint motors rotations will be passed to an esp32 using serial communciation, the esp32 will then move the motors to the desired positions 
 
 
The concept is simple:
The robot arm will scan for an object moving the arm gripper left and right
When a object is detected the arm will be moved left/rigt forward or backwards in order to center the object with the camera (mounted on the arm gripper) 
once the arm gripper is in position the arm will lower it , grab the object and move it to a predefined place.




## Getting started




## Testing docker containers communication


Open 2 terminals on one run this:

```
docker-compose run --rm moveit2

cd /shared_with_docker
python3 ros2_rx_test.py



```


In the other run this:

```
docker-compose run --rm yoloros2

cd /shared_with_docker
python3 ros2_tx_test.py
```





Now on the first terminal you should see :
```
[INFO] [1745316752.919949688] [dictionary_subscriber]: Name: ROS2 Node
[INFO] [1745316752.920668497] [dictionary_subscriber]: Value: 42
[INFO] [1745316752.921381160] [dictionary_subscriber]: Items: ['apple', 'banana', 'cherry']
[INFO] [1745316752.922081867] [dictionary_subscriber]: Nested value: value
[INFO] [1745316754.918837565] [dictionary_subscriber]: I received: {'name': 'ROS2 Node', 'value': 42, 'items': ['apple', 'banana', 'cherry'], 'nested': {'key': 'value'}}


```

it you see that message the communication is working correctly







## Build the arm configuration
To build the arm configuration run this command inside the moveit2 docker:

```
cd /shared_with_docker/ 
colcon build --packages-select robotic_arm7
```


For more info about the moveit2 configuration see the file moveit2_docker/moveit2_readme.md



## Run Rviz with the custom robotic arm configuration

Now to run the Rviz with the custom robotic arm configuration inside the moveit2 docker:

```

cd /shared_with_docker/ 
source install/setup.bash && source /opt/ros/rolling/setup.bash
ros2 launch robotic_arm7 demo.launch.py

```


This will display the custom created arm in Rviz, you can now controll the virtual arm moving the sphere with the mouse
For more info and if the procedure is not working see the file moveit2_docker/moveit2_readme.md





## Run the serial 232 interface software (control_servos_using_esp32_moveit2.py)
This software will receive the joint position from moveit2 using ros2 proptocol and the will ask the esp32 connected via serial port to command the servomotors


Now open a new terminal on the moveit2 docker:

I do this with this command:

```
docker exec -it $(docker ps | grep movit2_gpu_run | awk '{print $1}')   /bin/bash  && source /opt/ros/rolling/setup.bash

```


If you have not pyserial installed you could install it with:

```
cd /shared_with_docker/pyserial-3.5  && python3 setup.py install
```




Make sure to have the circuitpython software installed on the esp32 s3, look at How_to_install.MD  for more details.

Make sure you have connected the esp32 to the pc before running the docker-compose run --rm moveit2 otherwise the serial port will not be available

If that is the case close the docker, connect the esp32 and restart following the procedure since the start before continuing 
 
 
Run the serial interface software with:
```

python3 control_servos_using_esp32_moveit2.py

```



Now you should be able to move the virtual arm with Rviz and when you press Plan and execute the real arm should move too!
