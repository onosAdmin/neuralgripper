#include <memory>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

// Socket includes
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <cmath>  // For M_PI
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <iomanip>

/*
MoveIt2 Socket Server
Listens for commands like "x,10", "y,-5", "z,15" via TCP socket
Moves the robot end effector accordingly and returns new position/orientation

To run:
ros2 run axis_mover01 socket_server [port]
Default port is 8080

Commands: 
- "x,distance_mm", "y,distance_mm", "z,distance_mm" - Linear movement
- "rr,degrees", "rl,degrees" - Base rotation
- "movetofix,deg1,deg2,deg3,deg4,deg5,deg6" - Move all joints to specific degrees
- "get_pose" - Get current end effector pose
Response: "x,y,z,ox,oy,oz,ow" (position in meters, orientation as quaternion)
*/

class AxisMover {
public:
    AxisMover(const rclcpp::Node::SharedPtr& node)
        : node_(node)
    {
        try {
            RCLCPP_INFO(node_->get_logger(), "Creating MoveGroupInterface...");

            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                node_, "arm_group");

            RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface created successfully");

            RCLCPP_INFO(node_->get_logger(), "Waiting briefly for MoveIt internal state to update...");
            std::this_thread::sleep_for(std::chrono::seconds(3));

            RCLCPP_INFO(node_->get_logger(), "AxisMover initialization complete.");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception during AxisMover initialization: %s", e.what());
            throw;
        }
    }

    bool moveToJointPositions(const std::vector<double>& joint_degrees, std::string& response) {
        try {
            if (joint_degrees.size() != 6) {
                RCLCPP_ERROR(node_->get_logger(), "Invalid number of joint values. Expected 6, got %zu", joint_degrees.size());
                response = "ERROR: Must provide exactly 6 joint degree values";
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Moving joints to positions: [%.1f°, %.1f°, %.1f°, %.1f°, %.1f°, %.1f°]",
                       joint_degrees[0], joint_degrees[1], joint_degrees[2], 
                       joint_degrees[3], joint_degrees[4], joint_degrees[5]);

            // Convert degrees to radians
            std::vector<double> joint_radians(6);
            for (size_t i = 0; i < 6; ++i) {
                joint_radians[i] = joint_degrees[i] * M_PI / 180.0;
                RCLCPP_INFO(node_->get_logger(), "Joint %zu: %.1f° = %.4f rad", 
                           i, joint_degrees[i], joint_radians[i]);
            }

            // Set the joint target
            move_group_->setJointValueTarget(joint_radians);
            move_group_->setPlanningTime(15.0);  // Give more time for complex joint movements

            RCLCPP_INFO(node_->get_logger(), "Planning joint movement...");
            moveit::planning_interface::MoveGroupInterface::Plan plan;

            moveit::core::MoveItErrorCode plan_result;
            try {
                plan_result = move_group_->plan(plan);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Exception during planning: %s", e.what());
                response = "ERROR: Planning failed - " + std::string(e.what());
                return false;
            }

            if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "Planning successful, executing...");

                moveit::core::MoveItErrorCode execute_result;
                try {
                    execute_result = move_group_->execute(plan);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(), "Exception during execution: %s", e.what());
                    response = "ERROR: Execution failed - " + std::string(e.what());
                    return false;
                }

                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    // Get the new pose after movement
                    geometry_msgs::msg::PoseStamped new_pose = move_group_->getCurrentPose();
                    
                    // Also get current joint values to verify
                    std::vector<double> current_joints = move_group_->getCurrentJointValues();
                    RCLCPP_INFO(node_->get_logger(), "Movement complete. Current joint positions (degrees):");
                    for (size_t i = 0; i < current_joints.size() && i < 6; ++i) {
                        double degrees = current_joints[i] * 180.0 / M_PI;
                        RCLCPP_INFO(node_->get_logger(), "  Joint %zu: %.2f°", i, degrees);
                    }
                    
                    // Format response: x,y,z,ox,oy,oz,ow (position + quaternion orientation)
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(6)
                        << new_pose.pose.position.x << ","
                        << new_pose.pose.position.y << ","
                        << new_pose.pose.position.z << ","
                        << new_pose.pose.orientation.x << ","
                        << new_pose.pose.orientation.y << ","
                        << new_pose.pose.orientation.z << ","
                        << new_pose.pose.orientation.w;
                    
                    response = oss.str();
                    return true;
                } else {
                    response = "ERROR: Joint movement execution failed";
                    return false;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Motion planning failed with code: %d", plan_result.val);
                response = "ERROR: Motion planning failed - possibly invalid joint positions or collision detected";
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception during joint movement: %s", e.what());
            response = "ERROR: Exception during joint movement - " + std::string(e.what());
            return false;
        }
    }

    bool rotateBase(const std::string& direction, double degrees, std::string& response) {
        try {
            RCLCPP_INFO(node_->get_logger(), "Getting current joint values...");
            
            // Get current joint values
            std::vector<double> joint_values = move_group_->getCurrentJointValues();
            
            if (joint_values.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get current joint values");
                response = "ERROR: Failed to get current joint values";
                return false;
            }

            // Convert degrees to radians
            double radians = degrees * M_PI / 180.0;
            
            // Apply rotation direction (assuming rotating_base is the first joint - index 0)
            if (direction == "rr") {
                joint_values[0] += radians;  // Right rotation (positive)
            } else if (direction == "rl") {
                joint_values[0] -= radians;  // Left rotation (negative)
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Invalid rotation direction: '%s'. Must be 'rr' or 'rl'.", direction.c_str());
                response = "ERROR: Invalid rotation direction. Must be rr or rl";
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Target base joint angle: %.3f radians (%.3f degrees)", 
                       joint_values[0], joint_values[0] * 180.0 / M_PI);

            // Set the joint target
            move_group_->setJointValueTarget(joint_values);
            move_group_->setPlanningTime(10.0);

            RCLCPP_INFO(node_->get_logger(), "Planning base rotation...");
            moveit::planning_interface::MoveGroupInterface::Plan plan;

            moveit::core::MoveItErrorCode plan_result;
            try {
                plan_result = move_group_->plan(plan);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Exception during planning: %s", e.what());
                response = "ERROR: Planning failed - " + std::string(e.what());
                return false;
            }

            if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "Planning successful, executing...");

                moveit::core::MoveItErrorCode execute_result;
                try {
                    execute_result = move_group_->execute(plan);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(), "Exception during execution: %s", e.what());
                    response = "ERROR: Execution failed - " + std::string(e.what());
                    return false;
                }

                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    // Get the new pose after rotation
                    geometry_msgs::msg::PoseStamped new_pose = move_group_->getCurrentPose();
                    
                    // Format response: x,y,z,ox,oy,oz,ow (position + quaternion orientation)
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(6)
                        << new_pose.pose.position.x << ","
                        << new_pose.pose.position.y << ","
                        << new_pose.pose.position.z << ","
                        << new_pose.pose.orientation.x << ","
                        << new_pose.pose.orientation.y << ","
                        << new_pose.pose.orientation.z << ","
                        << new_pose.pose.orientation.w;
                    
                    response = oss.str();
                    return true;
                } else {
                    response = "ERROR: Rotation execution failed";
                    return false;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Motion planning failed with code: %d", plan_result.val);
                response = "ERROR: Motion planning failed";
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception during rotation: %s", e.what());
            response = "ERROR: Exception during rotation - " + std::string(e.what());
            return false;
        }
    }

    bool moveAlongAxis(const std::string& axis, double step_mm, std::string& response) {
        try {
            RCLCPP_INFO(node_->get_logger(), "Getting current state...");
            moveit::core::RobotStatePtr current_state;

            current_state = move_group_->getCurrentState(5.0);

            if (!current_state) {
                RCLCPP_ERROR(node_->get_logger(), "No current robot state available after timeout.");
                response = "ERROR: No current robot state available";
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Getting current pose...");
            geometry_msgs::msg::PoseStamped current_pose;

            current_pose = move_group_->getCurrentPose();

            if (current_pose.header.frame_id.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get current pose - empty frame ID");
                response = "ERROR: Failed to get current pose";
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Current position: x=%.3f, y=%.3f, z=%.3f",
                                             current_pose.pose.position.x,
                                             current_pose.pose.position.y,
                                             current_pose.pose.position.z);

            geometry_msgs::msg::Pose target_pose = current_pose.pose;
            double step = step_mm / 1000.0;  // Convert step from mm to meters

            if (axis == "x") target_pose.position.x += step;
            else if (axis == "y") target_pose.position.y += step;
            else if (axis == "z") target_pose.position.z += step;
            else {
                RCLCPP_ERROR(node_->get_logger(), "Invalid axis specified: '%s'. Must be 'x', 'y', or 'z'.", axis.c_str());
                response = "ERROR: Invalid axis. Must be x, y, or z";
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Target position: x=%.3f, y=%.3f, z=%.3f",
                                             target_pose.position.x, target_pose.position.y, target_pose.position.z);

            move_group_->setPoseTarget(target_pose);
            move_group_->setPlanningTime(10.0);

            RCLCPP_INFO(node_->get_logger(), "Planning movement...");
            moveit::planning_interface::MoveGroupInterface::Plan plan;

            moveit::core::MoveItErrorCode plan_result;
            try {
                plan_result = move_group_->plan(plan);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Exception during planning: %s", e.what());
                response = "ERROR: Planning failed - " + std::string(e.what());
                return false;
            }

            if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "Planning successful, executing...");

                moveit::core::MoveItErrorCode execute_result;
                try {
                    execute_result = move_group_->execute(plan);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(), "Exception during execution: %s", e.what());
                    response = "ERROR: Execution failed - " + std::string(e.what());
                    return false;
                }

                if (execute_result == moveit::core::MoveItErrorCode::SUCCESS) {
                    // Get the new pose after movement
                    geometry_msgs::msg::PoseStamped new_pose = move_group_->getCurrentPose();
                    
                    // Format response: x,y,z,ox,oy,oz,ow (position + quaternion orientation)
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(6)
                        << new_pose.pose.position.x << ","
                        << new_pose.pose.position.y << ","
                        << new_pose.pose.position.z << ","
                        << new_pose.pose.orientation.x << ","
                        << new_pose.pose.orientation.y << ","
                        << new_pose.pose.orientation.z << ","
                        << new_pose.pose.orientation.w;
                    
                    response = oss.str();
                    return true;
                } else {
                    response = "ERROR: Movement execution failed";
                    return false;
                }
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Motion planning failed with code: %d", plan_result.val);
                response = "ERROR: Motion planning failed";
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception during movement: %s", e.what());
            response = "ERROR: Exception during movement - " + std::string(e.what());
            return false;
        }
    }

    std::string getCurrentPose() {
        try {
            geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();
            
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(6)
                << current_pose.pose.position.x << ","
                << current_pose.pose.position.y << ","
                << current_pose.pose.position.z << ","
                << current_pose.pose.orientation.x << ","
                << current_pose.pose.orientation.y << ","
                << current_pose.pose.orientation.z << ","
                << current_pose.pose.orientation.w;
            
            return oss.str();
        } catch (const std::exception& e) {
            return "ERROR: Failed to get current pose - " + std::string(e.what());
        }
    }

    std::string getCurrentJoints() {
        try {
            std::vector<double> joint_values = move_group_->getCurrentJointValues();
            
            if (joint_values.empty()) {
                return "ERROR: Failed to get current joint values";
            }
            
            std::ostringstream oss;
            oss << std::fixed << std::setprecision(2);
            for (size_t i = 0; i < joint_values.size() && i < 6; ++i) {
                double degrees = joint_values[i] * 180.0 / M_PI;
                if (i > 0) oss << ",";
                oss << degrees;
            }
            
            return oss.str();
        } catch (const std::exception& e) {
            return "ERROR: Failed to get current joints - " + std::string(e.what());
        }
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

class SocketServer {
public:
    SocketServer(int port, std::shared_ptr<AxisMover> mover, const rclcpp::Node::SharedPtr& node)
        : port_(port), mover_(mover), node_(node), server_socket_(-1) {}

    ~SocketServer() {
        if (server_socket_ != -1) {
            close(server_socket_);
        }
    }

    bool start() {
        // Create socket
        server_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_socket_ == -1) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to create socket");
            return false;
        }

        // Set socket options to reuse address
        int opt = 1;
        if (setsockopt(server_socket_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set socket options");
            close(server_socket_);
            return false;
        }

        // Setup server address
        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_addr.s_addr = INADDR_ANY;
        server_addr.sin_port = htons(port_);

        // Bind socket
        if (bind(server_socket_, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to bind socket to port %d", port_);
            close(server_socket_);
            return false;
        }

        // Listen for connections
        if (listen(server_socket_, 5) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to listen on socket");
            close(server_socket_);
            return false;
        }

        RCLCPP_INFO(node_->get_logger(), "Socket server started on port %d", port_);
        return true;
    }

    void run() {
        while (rclcpp::ok()) {
            struct sockaddr_in client_addr;
            socklen_t client_len = sizeof(client_addr);
            
            RCLCPP_INFO(node_->get_logger(), "Waiting for client connection...");
            
            int client_socket = accept(server_socket_, (struct sockaddr*)&client_addr, &client_len);
            if (client_socket < 0) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to accept client connection");
                continue;
            }

            RCLCPP_INFO(node_->get_logger(), "Client connected: %s", inet_ntoa(client_addr.sin_addr));

            // Handle client in separate thread
            std::thread client_thread(&SocketServer::handleClient, this, client_socket);
            client_thread.detach();
        }
    }

private:
    void handleClient(int client_socket) {
        char buffer[1024];
        
        while (rclcpp::ok()) {
            memset(buffer, 0, sizeof(buffer));
            int bytes_received = recv(client_socket, buffer, sizeof(buffer) - 1, 0);
            
            if (bytes_received <= 0) {
                RCLCPP_INFO(node_->get_logger(), "Client disconnected");
                break;
            }

            std::string command(buffer);
            // Remove newline characters
            command.erase(std::remove(command.begin(), command.end(), '\n'), command.end());
            command.erase(std::remove(command.begin(), command.end(), '\r'), command.end());
            
            RCLCPP_INFO(node_->get_logger(), "Received command: '%s'", command.c_str());

            std::string response = processCommand(command);
            
            // Send response
            response += "\n";  // Add newline for easier parsing on client side
            send(client_socket, response.c_str(), response.length(), 0);
            
            RCLCPP_INFO(node_->get_logger(), "Sent response: '%s'", response.c_str());
        }
        
        close(client_socket);
    }

    std::string processCommand(const std::string& command) {
        // Check for special commands
        if (command == "get_pose" || command == "GET_POSE") {
            return mover_->getCurrentPose();
        }
        
        if (command == "get_joints" || command == "GET_JOINTS") {
            return mover_->getCurrentJoints();
        }

        // Check for movetofix command
        if (command.substr(0, 9) == "movetofix") {
            return processMoveToFixCommand(command);
        }

        // Parse regular command: type,value (e.g., "x,10", "rr,5", "rl,10")
        std::istringstream iss(command);
        std::string cmd_type;
        std::string value_str;
        
        if (!std::getline(iss, cmd_type, ',') || !std::getline(iss, value_str)) {
            return "ERROR: Invalid command format. Use 'type,value' (e.g., 'x,10', 'rr,5', 'rl,10') or 'movetofix,deg1,deg2,deg3,deg4,deg5,deg6'";
        }

        // Parse value
        double value;
        try {
            value = std::stod(value_str);
        } catch (const std::exception& e) {
            return "ERROR: Invalid value. Must be a number";
        }

        std::string response;
        
        // Check if it's a rotation command
        if (cmd_type == "rr" || cmd_type == "rl") {
            bool success = mover_->rotateBase(cmd_type, value, response);
            return response;
        }
        // Check if it's a linear movement command
        else if (cmd_type == "x" || cmd_type == "y" || cmd_type == "z") {
            bool success = mover_->moveAlongAxis(cmd_type, value, response);
            return response;
        }
        else {
            return "ERROR: Invalid command type. Use x,y,z for linear movement, rr,rl for base rotation, or movetofix for joint positions";
        }
    }

    std::string processMoveToFixCommand(const std::string& command) {
        // Expected format: "movetofix,deg1,deg2,deg3,deg4,deg5,deg6"
        std::istringstream iss(command);
        std::string token;
        std::vector<std::string> tokens;
        
        // Split by comma
        while (std::getline(iss, token, ',')) {
            tokens.push_back(token);
        }
        
        // Should have 7 tokens: "movetofix" + 6 degree values
        if (tokens.size() != 7) {
            return "ERROR: movetofix command requires exactly 6 joint degree values. Format: movetofix,deg1,deg2,deg3,deg4,deg5,deg6";
        }
        
        // Parse the degree values
        std::vector<double> joint_degrees(6);
        try {
            for (int i = 1; i < 7; ++i) {  // Skip first token which is "movetofix"
                joint_degrees[i-1] = std::stod(tokens[i]);
            }
        } catch (const std::exception& e) {
            return "ERROR: Invalid degree values. All joint degrees must be numbers";
        }
        
        // Execute the movement
        std::string response;
        bool success = mover_->moveToJointPositions(joint_degrees, response);
        return response;
    }

    int port_;
    std::shared_ptr<AxisMover> mover_;
    rclcpp::Node::SharedPtr node_;
    int server_socket_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Parse port argument
    int port = 8080;  // Default port
    if (argc >= 2) {
        try {
            port = std::stoi(argv[1]);
        } catch (const std::exception& e) {
            std::cerr << "Invalid port number: " << argv[1] << std::endl;
            rclcpp::shutdown();
            return 1;
        }
    }

    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<rclcpp::Node>("moveit_socket_server");
    executor.add_node(node);

    std::thread spinner_thread([&]() {
        executor.spin();
    });

    try {
        RCLCPP_INFO(node->get_logger(), "ROS and executor started. Waiting briefly for system to settle.");
        std::this_thread::sleep_for(std::chrono::seconds(3));

        std::shared_ptr<AxisMover> mover;
        try {
            mover = std::make_shared<AxisMover>(node);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(node->get_logger(), "Failed to initialize AxisMover: %s", e.what());
            executor.cancel();
            spinner_thread.join();
            rclcpp::shutdown();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "AxisMover initialized successfully.");

        // Create and start socket server
        SocketServer server(port, mover, node);
        if (!server.start()) {
            RCLCPP_FATAL(node->get_logger(), "Failed to start socket server");
            executor.cancel();
            spinner_thread.join();
            rclcpp::shutdown();
            return 1;
        }

        RCLCPP_INFO(node->get_logger(), "Socket server running. Use commands like:");
        RCLCPP_INFO(node->get_logger(), "  'x,10' - Move 10mm along X axis");
        RCLCPP_INFO(node->get_logger(), "  'y,-5' - Move -5mm along Y axis");
        RCLCPP_INFO(node->get_logger(), "  'z,15' - Move 15mm along Z axis");
        RCLCPP_INFO(node->get_logger(), "  'rr,10' - Rotate base 10 degrees to the right");
        RCLCPP_INFO(node->get_logger(), "  'rl,5' - Rotate base 5 degrees to the left");
        RCLCPP_INFO(node->get_logger(), "  'movetofix,270,90,95,180,15,22' - Move all joints to specific degrees");
        RCLCPP_INFO(node->get_logger(), "  'get_pose' - Get current end effector pose");
        RCLCPP_INFO(node->get_logger(), "  'get_joints' - Get current joint positions in degrees");

        // Run server (blocking call)
        server.run();

    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error during execution: %s", e.what());
    }

    executor.cancel();
    spinner_thread.join();

    rclcpp::shutdown();
    return 0;
}