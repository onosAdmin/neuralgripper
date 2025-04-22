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
