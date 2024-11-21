#include <iostream>
#include <vector>
#include <thread>
#include <cmath>
#include <chrono>
#include <mutex>
#include <functional>
#include <atomic>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include "unitree_sdk2cpp/core/channel.hpp"
#include "unitree_sdk2cpp/idl/default/unitree_go_msg_dds__SportModeState_.hpp"
#include "unitree_sdk2cpp/idl/unitree_go/msg/dds_/SportModeState_.hpp"
#include "unitree_sdk2cpp/go2/sport/sport_client.hpp"

using namespace unitree_sdk2cpp::idl::unitree_go::msg::dds_;
using namespace unitree_sdk2cpp::go2::sport;

class RobotController {
private:
    ros::NodeHandle nh;  // Node handle for ROS communication
    SportClient client;  // Client to communicate with the robot
    std::vector<double> initial_pos{0.0, 0.0, 0.0};  // Initial position of the robot
    std::vector<double> current_pos;  // Current position of the robot
    std::vector<double> target_pos_camera{0.0, 0.0, 0.0};  // Target position in camera frame
    double dt = 0.1;  // Time step for control loop
    double max_velocity = 0.2;  // Maximum velocity of the robot
    double min_distance = 0.1;  // Minimum distance to the target to stop
    double yaw = 0.0;  // Current yaw angle of the robot
    std::atomic<bool> stop_velocity_thread{false};  // Flag to stop the velocity update thread
    SportModeState_ robot_state;  // State of the robot
    std::thread velocity_thread;  // Thread for velocity updates
    std::mutex pos_mutex;  // Mutex to protect access to shared variables

    ros::Subscriber target_pos_sub;  // Subscriber for target position updates
    ros::Subscriber max_velocity_sub;  // Subscriber for max velocity updates
    ros::Publisher velocity_pub;  // Publisher for robot velocity commands

    // Callback function to update target position from ROS topic
    void targetPosCallback(const geometry_msgs::Point::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(pos_mutex);  // Lock mutex to safely update target position
        target_pos_camera = {msg->x, msg->y, msg->z};
        ROS_INFO("Received new target position: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
    }

    // Callback function to update max velocity from ROS topic
    void maxVelocityCallback(const std_msgs::Float64::ConstPtr& msg) {
        max_velocity = msg->data;
        ROS_INFO("Updated max velocity: %.2f", max_velocity);
    }

    // Transform target position from camera frame to robot frame
    std::vector<double> transform_camera_to_robot(const std::vector<double>& camera_pos) {
        // Transformation matrix to convert coordinates from camera to robot frame
        std::vector<std::vector<double>> R{
            {0, 0, 1},
            {-1, 0, 0},
            {0, -1, 0}
        };
        return {
            R[0][0] * camera_pos[0] + R[0][1] * camera_pos[1] + R[0][2] * camera_pos[2],
            R[1][0] * camera_pos[0] + R[1][1] * camera_pos[1] + R[1][2] * camera_pos[2],
            R[2][0] * camera_pos[0] + R[2][1] * camera_pos[1] + R[2][2] * camera_pos[2]
        };
    }

    // Calculate the yaw velocity needed to align with the target
    double calculate_yaw_velocity(double yaw_error, double k_p = 1.0) {
        double max_yaw_velocity = 0.5;  // Maximum allowable yaw velocity
        double yaw_velocity = k_p * yaw_error;  // Proportional control for yaw velocity
        return std::clamp(yaw_velocity, -max_yaw_velocity, max_yaw_velocity);  // Clamp to max yaw velocity
    }

    // Calculate linear and angular velocities to move towards the target
    std::vector<double> calculate_velocity() {
        std::lock_guard<std::mutex> lock(pos_mutex);  // Lock mutex to safely access current and target positions
        // Transform target position to robot frame
        auto target_pos_robot = transform_camera_to_robot(target_pos_camera);
        // Vector from current position to target position
        std::vector<double> vector_to_target{
            target_pos_robot[0] - current_pos[0],
            target_pos_robot[1] - current_pos[1]
        };

        // Calculate horizontal distance to the target
        double horizontal_distance = std::sqrt(std::pow(vector_to_target[0], 2) + std::pow(vector_to_target[1], 2));
        if (horizontal_distance < min_distance) {
            // If close enough to the target, stop moving
            return {0.0, 0.0, 0.0};
        }

        // Calculate direction towards the target
        double direction_x = vector_to_target[0] / horizontal_distance;
        double direction_y = vector_to_target[1] / horizontal_distance;
        double speed = std::min(horizontal_distance, max_velocity);  // Speed is limited by max velocity

        // Calculate velocities in x and y directions
        double vx = std::clamp(direction_x * speed, -max_velocity, max_velocity);
        double vy = std::clamp(direction_y * speed, -max_velocity, max_velocity);

        // Calculate desired yaw to face the target
        double desired_yaw = std::atan2(target_pos_robot[1] - current_pos[1], target_pos_robot[0] - current_pos[0]);
        double yaw_error = desired_yaw - yaw;

        // Normalize yaw error to the range [-pi, pi]
        if (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        else if (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        // Calculate yaw velocity to reduce yaw error
        double yaw_velocity = calculate_yaw_velocity(yaw_error);

        return {vx, vy, yaw_velocity};  // Return calculated velocities
    }

    // Update the current position of the robot based on velocities
    void update_position(const std::vector<double>& velocities) {
        std::lock_guard<std::mutex> lock(pos_mutex);  // Lock mutex to safely update current position
        current_pos[0] += velocities[0] * dt;  // Update x position
        current_pos[1] += velocities[1] * dt;  // Update y position
        yaw += velocities[2] * dt;  // Update yaw angle
    }

    // Thread function to continuously update robot velocities
    void velocity_update_thread_func() {
        ros::Rate rate(1.0 / dt);  // Set rate for control loop
        while (ros::ok() && !stop_velocity_thread) {
            // Calculate the velocities to reach the target
            auto velocities = calculate_velocity();
            {
                std::lock_guard<std::mutex> lock(pos_mutex);  // Lock mutex to safely access current and target positions
                // Calculate the distance to the target
                double distance_to_target = std::sqrt(
                    std::pow(current_pos[0] - target_pos_camera[0], 2) +
                    std::pow(current_pos[1] - target_pos_camera[1], 2)
                );

                // Stop if the robot is close enough to the target
                if (distance_to_target < min_distance) {
                    stop_velocity_thread = true;
                    ROS_INFO("Target reached. Stopping.");
                    break;
                }
            }

            // Send velocity command to the robot
            client.Move(velocities[0], velocities[1], velocities[2]);

            // Publish velocity command to ROS topic
            geometry_msgs::Twist vel_msg;
            vel_msg.linear.x = velocities[0];
            vel_msg.linear.y = velocities[1];
            vel_msg.angular.z = velocities[2];
            velocity_pub.publish(vel_msg);

            // Update the robot's current position
            update_position(velocities);
            rate.sleep();  // Sleep to maintain loop rate
        }

        // Stop the robot after reaching the target or stopping the thread
        client.StopMove();
        ROS_INFO("Velocity thread stopped.");
    }

public:
    // Constructor to initialize the robot controller
    RobotController() : nh("~") {
        client.SetTimeout(10.0);  // Set timeout for communication with the robot
        client.Init();  // Initialize the robot client
        current_pos = initial_pos;  // Set current position to initial position

        // Subscribe to target position and max velocity topics
        target_pos_sub = nh.subscribe("/target_position", 10, &RobotController::targetPosCallback, this);
        max_velocity_sub = nh.subscribe("/max_velocity", 10, &RobotController::maxVelocityCallback, this);
        // Advertise velocity commands on the /robot_velocity topic
        velocity_pub = nh.advertise<geometry_msgs::Twist> ("/robot_velocity", 10);
    }

    // Main function to run the robot controller
    void run() {
        ROS_INFO("Starting Robot Controller...");
        // Start the velocity update thread
        velocity_thread = std::thread(&RobotController::velocity_update_thread_func, this);
        velocity_thread.join();  // Wait for the thread to finish
        ROS_INFO("Robot Controller stopped.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_controller_node");  // Initialize ROS node

    // Initialize communication channel with optional argument
    if (argc > 1) {
        ChannelFactortyInitialize(0, argv[1]);
    } else {
        ChannelFactortyInitialize(0);
    }

    // Subscribe to robot state updates
    ChannelSubscriber sub("rt/sportmodestate", SportModeState_{});
    sub.Init([](const SportModeState_& msg) {
        ROS_INFO("Received robot state update.");
        // Handle the high state update if necessary
    }, 10);

    ros::Duration(1.0).sleep();  // Sleep for a short duration to ensure everything is initialized

    RobotController robot_controller;  // Create an instance of the robot controller
    robot_controller.run();  // Run the robot controller

    return 0;
}
