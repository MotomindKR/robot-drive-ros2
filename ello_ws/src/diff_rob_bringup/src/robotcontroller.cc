#include "robotcontroller.h"

RobotController::RobotController() : Node("robot_controller"), serial_fd(-1), x_pos(0.0), y_pos(0.0), theta(0.0), left_wheel_rpm(0.0), right_wheel_rpm(0.0), current_time(this->now()), last_time(this->now()), tf_broadcaster_(this) {
    serial_fd = open("/dev/ttyTHS0", O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd == -1) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
    } else {
        struct termios options;
        tcgetattr(serial_fd, &options);
        cfsetispeed(&options, B115200);
        cfsetospeed(&options, B115200);
        options.c_cflag |= (CLOCAL | CREAD);
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;
        tcsetattr(serial_fd, TCSANOW, &options);
    }

    cmd_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&RobotController::cmdVelCallback, this, std::placeholders::_1));

    odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    base_link_publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("/base_link", 10);
    
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RobotController::readSerial, this));
}

void RobotController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    double v = msg->linear.x;
    double w = msg->angular.z;
    
    double left_wheel_speed = (-1) * (2 * v - w * WHEEL_BASE) / (2 * (PI * WHEEL_DIAMETER));
    double right_wheel_speed = (2 * v + w * WHEEL_BASE) / (2 * (PI * WHEEL_DIAMETER));
    
    sendCommand("drv1 speed " + std::to_string(static_cast<int>(left_wheel_speed * 60)));
    sendCommand("drv1 start");
    sendCommand("drv2 speed " + std::to_string(static_cast<int>(right_wheel_speed * 60)));
    sendCommand("drv2 start");
}

void RobotController::sendCommand(const std::string &cmd) {
    if (serial_fd != -1) {
        write(serial_fd, (cmd + "\r\n").c_str(), cmd.length() + 2);
    }
}

void RobotController::readSerial() {
    char buffer[256];
    int bytes_read = read(serial_fd, buffer, sizeof(buffer) - 1);
    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        processSerialData(std::string(buffer));
    }
}


void RobotController::processSerialData(const std::string &data) {
    //RCLCPP_INFO(this->get_logger(), "Raw serial data: %s", data.c_str());
    std::stringstream ss(data);
    std::string item;
    std::vector<std::string> tokens;

    while (std::getline(ss, item, '\n')) {
        std::stringstream line_stream(item);
        std::vector<std::string> line_tokens;
        std::string token;

        while (std::getline(line_stream, token, ',')) {
            line_tokens.push_back(token);
        }

        if (line_tokens.size() < 3) continue;

        if (line_tokens[0] == "I" && line_tokens[1] == "enc1") {
            left_wheel_rpm = std::stod(line_tokens[2]);
            left_wheel_rpm = (-1) * left_wheel_rpm;
            //RCLCPP_INFO(this->get_logger(), "Received left RPM: %f", left_wheel_rpm);
        } else if (line_tokens[0] == "I" && line_tokens[1] == "enc2") {
            right_wheel_rpm = std::stod(line_tokens[2]);
            //right_wheel_rpm = (-1) * right_wheel_rpm;
            //RCLCPP_INFO(this->get_logger(), "Received right RPM: %f", right_wheel_rpm);
        }
    }
    updateOdometry();
}

void RobotController::updateOdometry() {
    double left_wheel_speed = (left_wheel_rpm / 60.0) * (PI * WHEEL_DIAMETER);
    double right_wheel_speed = (right_wheel_rpm / 60.0) * (PI * WHEEL_DIAMETER);
    double v = (left_wheel_speed + right_wheel_speed) / 2.0;
    double w = (right_wheel_speed - left_wheel_speed) / WHEEL_BASE;
    
    current_time = this->now();
    
    double dt = (current_time - last_time).seconds();
    theta += w * dt;
    x_pos += v * cos(theta) * dt;
    y_pos += v * sin(theta) * dt;
    
    RCLCPP_INFO(this->get_logger(), "Calculated velocity: Linear=%.4f m/s, Angular=%.4f rad/s", v, w);
    
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = this->now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_pos;
    odom.pose.pose.position.y = y_pos;
    odom.pose.pose.orientation.z = sin(theta / 2);
    odom.pose.pose.orientation.w = cos(theta / 2);
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    odom_pub->publish(odom);
    
    geometry_msgs::msg::TransformStamped odom_tf;
    odom_tf.header.stamp = this->now();
    
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_link";
    odom_tf.transform.translation.x = x_pos; 
    odom_tf.transform.translation.y = y_pos; 
    odom_tf.transform.translation.z = 0.0;
    
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    odom_tf.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_.sendTransform(odom_tf);
    base_link_publisher_->publish(odom_tf);   
    
    geometry_msgs::msg::TransformStamped base_footprint_tf;
    base_footprint_tf.header.stamp = this->now();
    base_footprint_tf.header.frame_id = "base_link";
    base_footprint_tf.child_frame_id = "base_footprint";
    base_footprint_tf.transform.translation.x = 0.0;
    base_footprint_tf.transform.translation.y = 0.0;
    base_footprint_tf.transform.translation.z = 0.0;
    base_footprint_tf.transform.rotation.x = 0.0;
    base_footprint_tf.transform.rotation.y = 0.0;
    base_footprint_tf.transform.rotation.z = 0.0;
    base_footprint_tf.transform.rotation.w = 1.0;
    
    tf_broadcaster_.sendTransform(base_footprint_tf);

    geometry_msgs::msg::TransformStamped lidar_tf;
    lidar_tf.header.stamp = this->now();
    lidar_tf.header.frame_id = "base_link";
    lidar_tf.child_frame_id = "lidar_frame";
    lidar_tf.transform.translation.x = 0.12;
    lidar_tf.transform.translation.y = 0.0;
    lidar_tf.transform.translation.z = 0.0;
    lidar_tf.transform.rotation.x = 0.0;
    lidar_tf.transform.rotation.y = 0.0;
    lidar_tf.transform.rotation.z = 1.0;
    lidar_tf.transform.rotation.w = 0.0;

    tf_broadcaster_.sendTransform(lidar_tf);

    last_time = current_time;


    tf_broadcaster_.sendTransform(base_footprint_tf);
    last_time = current_time;    
}

