
## Quick guide to build c++ packages for moveit2


Create the package:
```
ros2 pkg create --build-type ament_cmake --dependencies moveit_ros_planning_interface rclcpp --node-name hello_moveit_v0.1 --license BSD-3-Clause hello_moveit01
```


Build the package:

```

cd /shared_with_docker/  && colcon build --packages-select hello_moveit01
```



Source the package:

```

cd /shared_with_docker/  &&  source install/setup.bash && source /opt/ros/rolling/setup.bash
```



Run the package:

```

cd /shared_with_docker/  && ros2 run hello_moveit01 hello_moveit_v0.1
```



see this:
https://automaticaddison.com/how-to-create-your-first-c-moveit-2-project/
