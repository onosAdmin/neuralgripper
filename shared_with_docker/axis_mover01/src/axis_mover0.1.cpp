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
#include <fcntl.h>

// Required for argument parsing and string conversion
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <atomic>

class AxisMover {
public:
    AxisMover(const rclcpp::Node::SharedPtr& node)
        : node_(node), velocity_scaling_(0.1), acceleration_scaling_(0.1) // Default conservative values
    {
        try {
            RCLCPP_INFO(node_->get_logger(), "Creating MoveGroupInterface...");

            move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
                node_, "arm_group");

            RCLCPP_INFO(node_->get_logger(), "MoveGroupInterface created successfully");

            // Set initial scaling factors
            move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
            move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
            
            RCLCPP_INFO(node_->get_logger(), "Initial scaling factors set - Velocity: %.2f, Acceleration: %.2f", 
                       velocity_scaling_, acceleration_scaling_);

            RCLCPP_INFO(node_->get_logger(), "Waiting briefly for MoveIt internal state to update...");
            std::this_thread::sleep_for(std::chrono::seconds(5));

            RCLCPP_INFO(node_->get_logger(), "AxisMover initialization complete.");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception during AxisMover initialization: %s", e.what());
            throw;
        }
    }

    bool setScalingFactors(double velocity_scaling, double acceleration_scaling) {
        try {
            // Validate scaling factors (must be between 0.0 and 1.0)
            if (velocity_scaling < 0.0 || velocity_scaling > 1.0) {
                RCLCPP_ERROR(node_->get_logger(), "Invalid velocity scaling factor: %.3f. Must be between 0.0 and 1.0", velocity_scaling);
                return false;
            }
            
            if (acceleration_scaling < 0.0 || acceleration_scaling > 1.0) {
                RCLCPP_ERROR(node_->get_logger(), "Invalid acceleration scaling factor: %.3f. Must be between 0.0 and 1.0", acceleration_scaling);
                return false;
            }

            velocity_scaling_ = velocity_scaling;
            acceleration_scaling_ = acceleration_scaling;

            move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
            move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

            RCLCPP_INFO(node_->get_logger(), "Updated scaling factors - Velocity: %.3f, Acceleration: %.3f", 
                       velocity_scaling_, acceleration_scaling_);
            return true;

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception while setting scaling factors: %s", e.what());
            return false;
        }
    }

    bool moveAlongAxis(const std::string& axis, double step_mm) {
        try {
            RCLCPP_INFO(node_->get_logger(), "Getting current state...");
            moveit::core::RobotStatePtr current_state = move_group_->getCurrentState(5.0);

            if (!current_state) {
                RCLCPP_ERROR(node_->get_logger(), "No current robot state available after timeout.");
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Getting current pose...");
            geometry_msgs::msg::PoseStamped current_pose = move_group_->getCurrentPose();

            if (current_pose.header.frame_id.empty()) {
                RCLCPP_ERROR(node_->get_logger(), "Failed to get current pose - empty frame ID");
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
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Target position: x=%.3f, y=%.3f, z=%.3f (Vel: %.2f, Acc: %.2f)",
                                             target_pose.position.x, target_pose.position.y, target_pose.position.z,
                                             velocity_scaling_, acceleration_scaling_);

            move_group_->setPoseTarget(target_pose);
            move_group_->setPlanningTime(10.0);

            RCLCPP_INFO(node_->get_logger(), "Planning movement...");
            moveit::planning_interface::MoveGroupInterface::Plan plan;

            moveit::core::MoveItErrorCode plan_result;
            try {
                plan_result = move_group_->plan(plan);
            } catch (const std::exception& e) {
                RCLCPP_ERROR(node_->get_logger(), "Exception during planning: %s", e.what());
                return false;
            }

            if (plan_result == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(node_->get_logger(), "Planning successful, executing...");

                moveit::core::MoveItErrorCode execute_result;
                try {
                    execute_result = move_group_->execute(plan);
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(node_->get_logger(), "Exception during execution: %s", e.what());
                    return false;
                }

                return execute_result == moveit::core::MoveItErrorCode::SUCCESS;
            } else {
                RCLCPP_ERROR(node_->get_logger(), "Motion planning failed with code: %d", plan_result.val);
                return false;
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception during movement: %s", e.what());
            return false;
        }
    }

    double getVelocityScaling() const { return velocity_scaling_; }
    double getAccelerationScaling() const { return acceleration_scaling_; }

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    double velocity_scaling_;
    double acceleration_scaling_;
};

class SocketServer {
public:
    SocketServer(int port, std::shared_ptr<AxisMover> mover, const rclcpp::Node::SharedPtr& node) 
        : port_(port), mover_(mover), node_(node), running_(false) {}

    ~SocketServer() {
        stop();
    }

    bool start() {
        server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd_ == 0) {
            RCLCPP_ERROR(node_->get_logger(), "Socket creation failed");
            return false;
        }

        int opt = 1;
        if (setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
            RCLCPP_ERROR(node_->get_logger(), "Setsockopt failed");
            return false;
        }

        address_.sin_family = AF_INET;
        address_.sin_addr.s_addr = INADDR_ANY;
        address_.sin_port = htons(port_);

        if (bind(server_fd_, (struct sockaddr*)&address_, sizeof(address_)) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Bind failed on port %d", port_);
            return false;
        }

        if (listen(server_fd_, 3) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Listen failed");
            return false;
        }

        running_ = true;
        RCLCPP_INFO(node_->get_logger(), "Socket server started on port %d", port_);
        return true;
    }

    void stop() {
        running_ = false;
        if (server_fd_ >= 0) {
            close(server_fd_);
            server_fd_ = -1;
        }
    }

    void handleConnections() {
        int addrlen = sizeof(address_);
        
        while (running_) {
            int new_socket = accept(server_fd_, (struct sockaddr*)&address_, (socklen_t*)&addrlen);
            if (new_socket < 0) {
                if (running_) {
                    RCLCPP_ERROR(node_->get_logger(), "Accept failed");
                }
                continue;
            }

            RCLCPP_INFO(node_->get_logger(), "New client connected");
            
            // Handle client in separate thread to allow multiple connections
            std::thread client_thread(&SocketServer::handleClient, this, new_socket);
            client_thread.detach();
        }
    }

private:
    void handleClient(int client_socket) {
        char buffer[1024] = {0};
        
        while (running_) {
            int valread = read(client_socket, buffer, 1024);
            if (valread <= 0) {
                break;  // Client disconnected
            }

            buffer[valread] = '\0';
            std::string command(buffer);
            
            // Remove newline characters
            command.erase(std::remove(command.begin(), command.end(), '\n'), command.end());
            command.erase(std::remove(command.begin(), command.end(), '\r'), command.end());
            
            RCLCPP_INFO(node_->get_logger(), "Received command: '%s'", command.c_str());
            
            std::string response = processCommand(command);
            send(client_socket, response.c_str(), response.length(), 0);
        }
        
        close(client_socket);
        RCLCPP_INFO(node_->get_logger(), "Client disconnected");
    }

    std::string processCommand(const std::string& command) {
        // Parse comma-separated command format: "type,value" or "scaling,velocity,acceleration"
        std::vector<std::string> tokens;
        std::stringstream ss(command);
        std::string token;
        
        while (std::getline(ss, token, ',')) {
            tokens.push_back(token);
        }

        if (tokens.empty()) {
            return "ERROR: Empty command\n";
        }

        std::string cmd_type = tokens[0];
        
        // Handle axis movement commands: x,10 | y,-5 | z,20
        if ((cmd_type == "x" || cmd_type == "y" || cmd_type == "z") && tokens.size() == 2) {
            double distance;
            
            try {
                distance = std::stod(tokens[1]);
            } catch (const std::exception& e) {
                return "ERROR: Invalid distance parameter\n";
            }

            if (mover_->moveAlongAxis(cmd_type, distance)) {
                return "SUCCESS: Movement completed\n";
            } else {
                return "ERROR: Movement failed\n";
            }
        }
        // Handle scaling command: scaling,velocity,acceleration
        else if (cmd_type == "scaling" && tokens.size() == 3) {
            double velocity, acceleration;
            
            try {
                velocity = std::stod(tokens[1]);
                acceleration = std::stod(tokens[2]);
            } catch (const std::exception& e) {
                return "ERROR: Invalid scaling parameters\n";
            }

            if (mover_->setScalingFactors(velocity, acceleration)) {
                return "SUCCESS: Scaling factors updated\n";
            } else {
                return "ERROR: Failed to set scaling factors\n";
            }
        }
        // Handle velocity scaling only: vel,0.5
        else if (cmd_type == "vel" && tokens.size() == 2) {
            double velocity;
            
            try {
                velocity = std::stod(tokens[1]);
            } catch (const std::exception& e) {
                return "ERROR: Invalid velocity parameter\n";
            }

            if (mover_->setScalingFactors(velocity, mover_->getAccelerationScaling())) {
                return "SUCCESS: Velocity scaling updated\n";
            } else {
                return "ERROR: Failed to set velocity scaling\n";
            }
        }
        // Handle acceleration scaling only: acc,0.3
        else if (cmd_type == "acc" && tokens.size() == 2) {
            double acceleration;
            
            try {
                acceleration = std::stod(tokens[1]);
            } catch (const std::exception& e) {
                return "ERROR: Invalid acceleration parameter\n";
            }

            if (mover_->setScalingFactors(mover_->getVelocityScaling(), acceleration)) {
                return "SUCCESS: Acceleration scaling updated\n";
            } else {
                return "ERROR: Failed to set acceleration scaling\n";
            }
        }
        // Handle status command
        else if (cmd_type == "status") {
            std::ostringstream oss;
            oss << "STATUS: Velocity=" << mover_->getVelocityScaling() 
                << " Acceleration=" << mover_->getAccelerationScaling() << "\n";
            return oss.str();
        }
        // Handle help command
        else if (cmd_type == "help") {
            return "COMMANDS:\n"
                   "  x,<distance_mm>           - Move along x-axis\n"
                   "  y,<distance_mm>           - Move along y-axis\n"
                   "  z,<distance_mm>           - Move along z-axis\n"
                   "  scaling,<vel>,<acc>       - Set both scaling factors (0.0-1.0)\n"
                   "  vel,<velocity>            - Set velocity scaling only\n"
                   "  acc,<acceleration>        - Set acceleration scaling only\n"
                   "  status                    - Get current scaling factors\n"
                   "  help                      - Show this help\n";
        }
        else {
            return "ERROR: Invalid command format. Use 'type,value' (e.g., 'x,10', 'scaling,0.5,0.3') or 'help'\n";
        }
    }

    int port_;
    int server_fd_ = -1;
    struct sockaddr_in address_;
    std::shared_ptr<AxisMover> mover_;
    rclcpp::Node::SharedPtr node_;
    std::atomic<bool> running_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // Parse arguments - now optional for socket mode
    std::string axis_arg;
    double distance_mm = 0.0;
    int socket_port = 8080;  // Default port
    bool socket_mode = true;  // Default to socket mode

    if (argc == 3) {
        // Command line mode
        axis_arg = argv[1];
        socket_mode = false;
        
        if (axis_arg != "x" && axis_arg != "y" && axis_arg != "z") {
            RCLCPP_FATAL(rclcpp::get_logger("main"), "Invalid axis specified: '%s'. Must be 'x', 'y', or 'z'.", axis_arg.c_str());
            rclcpp::shutdown();
            return 1;
        }

        try {
            distance_mm = std::stod(argv[2]);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(rclcpp::get_logger("main"), "Invalid distance argument: '%s'", argv[2]);
            rclcpp::shutdown();
            return 1;
        }
    } else if (argc == 2) {
        // Socket mode with custom port
        try {
            socket_port = std::stoi(argv[1]);
        } catch (const std::exception& e) {
            RCLCPP_FATAL(rclcpp::get_logger("main"), "Invalid port argument: '%s'", argv[1]);
            rclcpp::shutdown();
            return 1;
        }
    } else if (argc != 1) {
        std::cerr << "Usage options:" << std::endl;
        std::cerr << "  " << argv[0] << "                    - Socket mode (port 8080)" << std::endl;
        std::cerr << "  " << argv[0] << " <port>            - Socket mode (custom port)" << std::endl;
        std::cerr << "  " << argv[0] << " <axis> <dist_mm>  - Single command mode" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<rclcpp::Node>("axis_mover");
    executor.add_node(node);

    std::thread spinner_thread([&]() {
        executor.spin();
    });

    try {
        RCLCPP_INFO(node->get_logger(), "ROS and executor started. Waiting briefly for system to settle.");
        std::this_thread::sleep_for(std::chrono::seconds(5));

        auto mover = std::make_shared<AxisMover>(node);
        RCLCPP_INFO(node->get_logger(), "AxisMover initialized successfully.");

        if (socket_mode) {
            RCLCPP_INFO(node->get_logger(), "Starting socket server on port %d", socket_port);
            SocketServer server(socket_port, mover, node);
            
            if (server.start()) {
                RCLCPP_INFO(node->get_logger(), "Socket server ready. Use commands like:");
                RCLCPP_INFO(node->get_logger(), "  'move x 10' - Move 10mm along x-axis");
                RCLCPP_INFO(node->get_logger(), "  'scaling 0.5 0.3' - Set velocity=0.5, acceleration=0.3");
                RCLCPP_INFO(node->get_logger(), "  'status' - Get current scaling factors");
                
                server.handleConnections();
            } else {
                RCLCPP_ERROR(node->get_logger(), "Failed to start socket server");
            }
        } else {
            // Single command mode
            RCLCPP_INFO(node->get_logger(), "Attempting to move %.2f mm along the '%s' axis.", distance_mm, axis_arg.c_str());
            
            if (mover->moveAlongAxis(axis_arg, distance_mm)) {
                RCLCPP_INFO(node->get_logger(), "Movement succeeded");
            } else {
                RCLCPP_ERROR(node->get_logger(), "Movement failed");
            }
        }

    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error during execution: %s", e.what());
    }

    executor.cancel();
    spinner_thread.join();
    rclcpp::shutdown();
    return 0;
}
