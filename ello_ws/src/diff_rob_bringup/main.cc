#include "rclcpp/rclcpp.hpp"
#include "robotcontroller.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotController>());
    rclcpp::shutdown();
    return 0;
}

