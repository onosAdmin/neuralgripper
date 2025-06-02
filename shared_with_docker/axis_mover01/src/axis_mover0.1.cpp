#include <memory>
#include <chrono>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp> // Required for executors
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp> // Required for PoseStamped

// Required for argument parsing and string conversion
#include <string>
#include <vector>
#include <iostream>
#include <stdexcept> // Required for stod exceptions



/*
make a moveit2 c++ code  where there is a class that given a axis (x, y or z) and a step move in mm 
it will read the current joint position and  move the end effector in the axis requested by the step mm  given, keeping the current end effector pose

To run use:
ros2 run axis_mover01 axis_mover0.1 x 3
ros2 run axis_mover01 axis_mover0.1 x -2

ros2 run axis_mover01 axis_mover0.1 y 2
ros2 run axis_mover01 axis_mover0.1 y -2

ros2 run axis_mover01 axis_mover0.1 z 2
ros2 run axis_mover01 axis_mover0.1 z -2

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
            // Increased initial wait for robust startup
            std::this_thread::sleep_for(std::chrono::seconds(5));

            RCLCPP_INFO(node_->get_logger(), "AxisMover initialization complete.");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception during AxisMover initialization: %s", e.what());
            throw; // Re-throw the exception to indicate initialization failure
        }
    }

    bool moveAlongAxis(const std::string& axis, double step_mm) {
        try {
            RCLCPP_INFO(node_->get_logger(), "Getting current state...");
            moveit::core::RobotStatePtr current_state;

            current_state = move_group_->getCurrentState(5.0);

            if (!current_state) {
                RCLCPP_ERROR(node_->get_logger(), "No current robot state available after timeout.");
                return false;
            }

            RCLCPP_INFO(node_->get_logger(), "Getting current pose...");
            geometry_msgs::msg::PoseStamped current_pose;

            current_pose = move_group_->getCurrentPose();

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

private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    // --- Argument Parsing ---
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <axis> <distance_mm>" << std::endl;
        std::cerr << "  <axis>: The axis to move along ('x', 'y', or 'z')" << std::endl;
        std::cerr << "  <distance_mm>: The distance to move in millimeters (e.g., 10.0 or -5)" << std::endl;
        rclcpp::shutdown();
        return 1; // Indicate error
    }

    std::string axis_arg = argv[1];
    double distance_mm;

    if (axis_arg != "x" && axis_arg != "y" && axis_arg != "z") {
         RCLCPP_FATAL(rclcpp::get_logger("main"), "Invalid axis specified: '%s'. Must be 'x', 'y', or 'z'.", axis_arg.c_str());
         rclcpp::shutdown();
         return 1; // Indicate error
    }

    try {
        distance_mm = std::stod(argv[2]);
    } catch (const std::invalid_argument& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Invalid argument for distance: '%s' is not a valid number.", argv[2]);
        rclcpp::shutdown();
        return 1; // Indicate error
    } catch (const std::out_of_range& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Invalid argument for distance: '%s' is out of range.", argv[2]);
        rclcpp::shutdown();
        return 1; // Indicate error
    }
    // --- End Argument Parsing ---


    rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<rclcpp::Node>("axis_mover");
    executor.add_node(node);

    std::thread spinner_thread([&]() {
        executor.spin();
    });

    try {
        RCLCPP_INFO(node->get_logger(), "ROS and executor started. Waiting briefly for system to settle.");
        std::this_thread::sleep_for(std::chrono::seconds(5));

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
        RCLCPP_INFO(node->get_logger(), "Attempting to move %.2f mm along the '%s' axis.", distance_mm, axis_arg.c_str());


        // --- Call moveAlongAxis with parsed arguments ---
        if (mover->moveAlongAxis(axis_arg, distance_mm)) {
            RCLCPP_INFO(node->get_logger(), "Movement succeeded");
        } else {
            RCLCPP_ERROR(node->get_logger(), "Movement failed");
        }
        // --- End Call ---


        RCLCPP_INFO(node->get_logger(), "Movement attempt finished. Shutting down...");

    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("main"), "Fatal error during execution: %s", e.what());
    }

    executor.cancel();
    spinner_thread.join();

    rclcpp::shutdown();
    return 0;
}
