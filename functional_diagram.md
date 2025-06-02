


The system works like this:

There is the moveit2 server (ros2 launch robotic_arm7 demo.launch.py) 
that will show the robot arm interface and receive planning request, 
once a planning request is received it will calculate if the destination position is achievable,
if is it will move the joints state gradually to that position.


There is the esp32 interface (python3 control_servos_using_esp32_moveit2.py)
That will keep reading the robots joints values using ros2 protocol and will traslate the message to the esp32 using the serial port 



There is the arm orchestrator (ros2 run socket_arm_mover01 socket_arm_mover_v0.1)
That will receive the object position from yolo_class_direction_provider_publisher.py and then decide what to , moving the arm using the socket_arm_mover01


There is socket_arm_mover01 that will receive the move the orchestrator wants to do and then execute it if possible. 
This code will read the current robot position from ros2 and will then ask moveit2 for the planning and execution of the move.
The move in x y and z are usually relative the current position for example move the arm gripper 2 cm up, or move the gripper 3 cm far away from the base of the arm


There is the yolo_class_direction_provider_publisher.py that will read the camera, use yolo to detect the brick and then send the data to arm orchestrator
sending the position of the lego brick and if the position is centered  or not


There is the code.py running on esp32 that will read the received command and move the servos to the requested positions






graph TD
    %% Vision System
    A[Camera] --> B[yolo_class_direction_provider_publisher.py]
    B --> |"Object position<br/>& centering status"| C[Arm Orchestrator<br/>socket_arm_mover01]
    
    %% Decision and Planning Layer
    C --> |"Movement commands<br/>(relative positions)"| D[socket_arm_mover01]
    D --> |"Read current<br/>robot position"| E[MoveIt2 Server<br/>demo.launch.py]
    D --> |"Planning request<br/>(destination position)"| E
    
    %% Motion Planning and Execution
    E --> |"Calculate if position<br/>is achievable"| E
    E --> |"Joint states<br/>(gradual movement)"| F[ESP32 Interface<br/>control_servos_using_esp32_moveit2.py]
    
    %% Hardware Interface
    F --> |"ROS2 protocol<br/>(read joint values)"| E
    F --> |"Serial communication<br/>(translated commands)"| G[ESP32 Hardware]
    G --> |"Servo control"| H[Physical Robot Arm]
    
    %% Visual Interface
    E --> I[Robot Arm Interface<br/>Visual Display]
    
    %% Styling
    classDef vision fill:#e1f5fe,stroke:#01579b,stroke-width:2px
    classDef planning fill:#f3e5f5,stroke:#4a148c,stroke-width:2px
    classDef execution fill:#e8f5e8,stroke:#1b5e20,stroke-width:2px
    classDef hardware fill:#fff3e0,stroke:#e65100,stroke-width:2px
    classDef interface fill:#fce4ec,stroke:#880e4f,stroke-width:2px
    
    class A,B vision
    class C,D,E planning
    class F execution
    class G,H hardware
    class I interface
    
    %% Notes
    subgraph Legend
        L1[Vision & Detection]:::vision
        L2[Planning & Decision]:::planning  
        L3[Execution Control]:::execution
        L4[Hardware Layer]:::hardware
        L5[User Interface]:::interface
    end

