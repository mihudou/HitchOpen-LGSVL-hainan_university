#ifndef SIMPLE_CONTROL__CONTROLLER_MANAGER_NODE_HPP_
#define SIMPLE_CONTROL__CONTROLLER_MANAGER_NODE_HPP_

#include <memory>
#include <vector>
#include <string>

#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "simple_control/controller_interface.hpp"

#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/bool.hpp"
#include "race_msgs/msg/vehicle_flag.hpp"
#include "race_msgs/msg/can.hpp"

namespace control
{
class ControllerManagerNode : public rclcpp::Node
{
public:
    ControllerManagerNode();
    void addController(const std::string& name, const std::string& type);
    void runStep();

private:
    // Callbacks
    void should_publish_control_callback(const std_msgs::msg::Bool::SharedPtr msg);
    void vehicle_flag_callback(const race_msgs::msg::VehicleFlag::SharedPtr msg);
    void on_local_path_callback(const autoware_planning_msgs::msg::Path::SharedPtr msg);
    void on_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void on_can_callback(const race_msgs::msg::CAN::SharedPtr msg);
    void checkFlagTimeout();

    // Utility
    double calculate_heading_error();
    double normalize_angle(double angle);

    // ROS interfaces
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr should_publish_control_sub_;
    rclcpp::Subscription<race_msgs::msg::VehicleFlag>::SharedPtr vehicle_flag_sub_;
    rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr local_path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<race_msgs::msg::CAN>::SharedPtr can_sub_;
    rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    rclcpp::TimerBase::SharedPtr flag_check_timer_;

    // Parameters and internal state
    double flag_timeout_ = 1.0;
    rclcpp::Time last_flag_time_;
    bool should_publish_control_ = true;
    uint8_t vehicle_flag_ = race_msgs::msg::VehicleFlag::RED;

    autoware_planning_msgs::msg::Path::SharedPtr local_path_;
    nav_msgs::msg::Odometry::SharedPtr odom_;

    pluginlib::ClassLoader<control::ControllerInterface> m_plugin_loader_;
    std::vector<control::ControllerInterface::SharedPtr> m_plugins_;

    // Gear management
    double current_rpm_ = 0.0;
    int current_gear_ = 1;
    rclcpp::Time last_shift_time_;
    double rpm_threshold_ = 6000.0;
    int max_gear_ = 6;
    double heading_tolerance_rad_ = 0.05; // about 2.8 deg
    rclcpp::Duration shift_cooldown_ = rclcpp::Duration::from_seconds(1.0);
};

}  // namespace control

#endif  // SIMPLE_CONTROL__CONTROLLER_MANAGER_NODE_HPP_

