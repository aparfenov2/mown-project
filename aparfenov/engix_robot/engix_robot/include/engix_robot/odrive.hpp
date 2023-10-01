#ifndef ODRIVE_HPP_
#define ODRIVE_HPP_

#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <libusb-1.0/libusb.h>
#include "engix_robot/odrive_msg.h"
#include "engix_robot/odrive_ctrl.h"
#include <string>
#include <fstream>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "tf/transform_broadcaster.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/BatteryState.h"
#include "engix_robot/odrive_endpoint.hpp"
#include "engix_robot/odrive_utils.hpp"
#include "engix_robot/odrive_enums.hpp"
#include <jsoncpp/json/json.h>

#include <diagnostic_updater/diagnostic_updater.h>
#include <std_msgs/Bool.h>
#include <diagnostic_updater/publisher.h>

#define ODRIVE_OK    0
#define ODRIVE_ERROR 1

// Listener commands
enum commands {
    CMD_AXIS_RESET,
    CMD_AXIS_IDLE,
    CMD_AXIS_CLOSED_LOOP,
    CMD_AXIS_SET_VELOCITY,
    CMD_REBOOT
};

class odrive{

    private:
        void msgCallback(const engix_robot::odrive_ctrl::ConstPtr& msg);
        void velCallback(const geometry_msgs::Twist &vel);
    public:

};
#endif
