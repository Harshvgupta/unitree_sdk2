// robot_controller.cpp
#include "robot_controller.hpp"

using namespace unitree_sdk2cpp::idl::unitree_go::msg::dds_;
using namespace unitree_sdk2cpp::go2::sport;

// Constructor to initialize the robot controller
RobotController::RobotController() : nh("~") {
    client.SetTimeout(10.0);  // Set timeout for communication with the robot
    client.Init();  // Initialize the robot client
    current_pos = initial_pos;  // Set current position to initial position

    // Subscribe to target position and max velocity topics
    target_pos_sub = nh.subscribe("/target_position", 10, &RobotController::targetPosCallback, this);
    max_velocity_sub = nh.subscribe("/max_velocity", 10, &RobotController::maxVelocityCallback, this);
    // Advertise velocity commands on the /robot_velocity topic
    velocity_pub = nh.advertise<geometry_msgs::Twist> ("/robot_velocity", 10);
}

// Callback function to update target position from ROS topic
void RobotController::targetPosCallback(const geometry_msgs::Point::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(pos_mutex);  // Lock mutex to safely update target position
    target_pos_camera = {msg->x, msg->y, msg->z};
    ROS_INFO("Received new target position: [%.2f, %.2f, %.2f]", msg->x, msg->y, msg->z);
}

// Callback function to update max velocity from ROS topic
void RobotController::maxVelocityCallback(const std_msgs::Float64::ConstPtr& msg) {
    max_velocity = msg->data;
    ROS_INFO("Updated max velocity: %.2f", max_velocity);
}

// Transform target position from camera frame to robot frame
std::vector<double> RobotController::transform_camera_to_robot(const std::vector<double>& camera_pos) {
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
double RobotController::calculate_yaw_velocity(double yaw_error, double k_p) {
    double max_yaw_velocity = 0.5;  // Maximum allowable yaw velocity
    double yaw_velocity = k_p * yaw_error;  // Proportional control for yaw velocity
    return std::clamp(yaw_velocity, -max_yaw_velocity, max_yaw_velocity);  // Clamp to max yaw velocity
}

// Calculate linear and angular velocities to move towards the target
std::vector<double> RobotController::calculate_velocity() {
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
void RobotController::update_position(const std::vector<double>& velocities) {
    std::lock_guard<std::mutex> lock(pos_mutex);  // Lock mutex to safely update current position
    current_pos[0] += velocities[0] * dt;  // Update x position
    current_pos[1] += velocities[1] * dt;  // Update y position
    yaw += velocities[2] * dt;  // Update yaw angle
}

// Request to stop the velocity update thread
void RobotController::request_stop() {
    std::lock_guard<std::mutex> lock(pos_mutex);
    stop_requested = true;
    stop_cv.notify_all();
}

// Thread function to continuously update robot velocities
void RobotController::velocity_update_thread_func() {
    ros::Rate rate(1.0 / dt);  // Set rate for control loop
    std::unique_lock<std::mutex> lock(pos_mutex);
    while (ros::ok() && !stop_requested) {
        auto velocities = calculate_velocity();

        // Calculate the distance to the target
        double distance_to_target = std::sqrt(
            std::pow(current_pos[0] - target_pos_camera[0], 2) +
            std::pow(current_pos[1] - target_pos_camera[1], 2)
        );

        // Stop if the robot is close enough to the target
        if (distance_to_target < min_distance) {
            request_stop();
            ROS_INFO("Target reached. Stopping.");
            break;
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

// Main function to run the robot controller
void RobotController::run() {
    ROS_INFO("Starting Robot Controller...");
    // Start the velocity update thread
    velocity_thread = std::thread(&RobotController::velocity_update_thread_func, this);
    velocity_thread.join();  // Wait for the thread to finish
    ROS_INFO("Robot Controller stopped.");
}
