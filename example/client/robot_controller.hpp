// robot_controller.hpp
#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include <vector>
#include <thread>
#include <cmath>
#include <chrono>
#include <mutex>
#include <functional>
#include <atomic>
#include <condition_variable>
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
    bool stop_requested = false;  // Flag to stop the velocity update thread
    SportModeState_ robot_state;  // State of the robot
    std::thread velocity_thread;  // Thread for velocity updates
    std::mutex pos_mutex;  // Mutex to protect access to shared variables
    std::condition_variable stop_cv;  // Condition variable to signal thread stop

    ros::Subscriber target_pos_sub;  // Subscriber for target position updates
    ros::Subscriber max_velocity_sub;  // Subscriber for max velocity updates
    ros::Publisher velocity_pub;  // Publisher for robot velocity commands

    // Callback function to update target position from ROS topic
    void targetPosCallback(const geometry_msgs::Point::ConstPtr& msg);

    // Callback function to update max velocity from ROS topic
    void maxVelocityCallback(const std_msgs::Float64::ConstPtr& msg);

    // Transform target position from camera frame to robot frame
    std::vector<double> transform_camera_to_robot(const std::vector<double>& camera_pos);

    // Calculate the yaw velocity needed to align with the target
    double calculate_yaw_velocity(double yaw_error, double k_p = 1.0);

    // Calculate linear and angular velocities to move towards the target
    std::vector<double> calculate_velocity();

    // Update the current position of the robot based on velocities
    void update_position(const std::vector<double>& velocities);

    // Thread function to continuously update robot velocities
    void velocity_update_thread_func();

    // Request to stop the velocity update thread
    void request_stop();

public:
    // Constructor to initialize the robot controller
    RobotController();

    // Main function to run the robot controller
    void run();
};

#endif
