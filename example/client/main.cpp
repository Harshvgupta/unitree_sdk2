// main.cpp
#include "robot_controller.hpp"
#include <ros/ros.h>
#include "unitree_sdk2cpp/core/channel.hpp"
#include "unitree_sdk2cpp/idl/default/unitree_go_msg_dds__SportModeState_.hpp"
#include "unitree_sdk2cpp/idl/unitree_go/msg/dds_/SportModeState_.hpp"

using namespace unitree_sdk2cpp::idl::unitree_go::msg::dds_;

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
