#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#include <chrono>
#include <thread>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <signal.h>
#include <cmath>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sstream>
#include <array>
#include <algorithm>
#include <atomic>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For tf2::toMsg and tf2::Quaternion

#define WHEEL_DIAMETER 0.127 // 5 inches in meters
#define WHEEL_BASE 0.375     // Distance between wheels in meters
#define PI 3.14159265359

class RobotController : public rclcpp::Node {
public:
    RobotController();

private:
    int serial_fd;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
    rclcpp::TimerBase::SharedPtr timer_;
    
    rclcpp::Time current_time;
    rclcpp::Time last_time;
    double x_pos, y_pos, theta;
    double left_wheel_rpm, right_wheel_rpm;
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void sendCommand(const std::string &cmd);
    void readSerial();
    void processSerialData(const std::string &data);
    void updateOdometry();
    rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr base_link_publisher_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;
};

#endif

