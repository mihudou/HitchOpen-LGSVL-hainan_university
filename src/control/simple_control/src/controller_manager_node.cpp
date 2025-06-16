#include "simple_control/controller_manager_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <cmath>

namespace control
{

ControllerManagerNode::ControllerManagerNode()
    : Node("controller_manager"),
      m_plugin_loader_("simple_control", "control::ControllerInterface")
{
    this->declare_parameter("manager.debug", false);
    this->declare_parameter("manager.flag_timeout", 1.0);
    this->declare_parameter("manager.rpm_threshold", 6000.0);
    this->declare_parameter("manager.shift_cooldown", 1.0);
    this->declare_parameter("manager.max_gear", 6);
    this->declare_parameter("manager.heading_tolerance_deg", 3.0);

    flag_timeout_ = this->get_parameter("manager.flag_timeout").as_double();
    rpm_threshold_ = this->get_parameter("manager.rpm_threshold").as_double();
    max_gear_ = this->get_parameter("manager.max_gear").as_int();
    double cooldown = this->get_parameter("manager.shift_cooldown").as_double();
    double heading_tol_deg = this->get_parameter("manager.heading_tolerance_deg").as_double();
    heading_tolerance_rad_ = heading_tol_deg * M_PI / 180.0;
    shift_cooldown_ = rclcpp::Duration::from_seconds(cooldown);
    last_shift_time_ = this->now();

    control_pub_ = this->create_publisher<autoware_control_msgs::msg::Control>("control_cmd", 10);
    should_publish_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "should_publish_control", 10, std::bind(&ControllerManagerNode::should_publish_control_callback, this, std::placeholders::_1));
    vehicle_flag_sub_ = this->create_subscription<race_msgs::msg::VehicleFlag>(
        "vehicle_flag", 10, std::bind(&ControllerManagerNode::vehicle_flag_callback, this, std::placeholders::_1));
    local_path_sub_ = this->create_subscription<autoware_planning_msgs::msg::Path>(
        "local_path", 10, std::bind(&ControllerManagerNode::on_local_path_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&ControllerManagerNode::on_odom_callback, this, std::placeholders::_1));
    can_sub_ = this->create_subscription<race_msgs::msg::CAN>(
        "can", 10, std::bind(&ControllerManagerNode::on_can_callback, this, std::placeholders::_1));

    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControllerManagerNode::runStep, this));
    flag_check_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&ControllerManagerNode::checkFlagTimeout, this));
}

void ControllerManagerNode::should_publish_control_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
    should_publish_control_ = msg->data;
}

void ControllerManagerNode::vehicle_flag_callback(const race_msgs::msg::VehicleFlag::SharedPtr msg)
{
    vehicle_flag_ = msg->flag;
    last_flag_time_ = this->now();
}

void ControllerManagerNode::on_local_path_callback(const autoware_planning_msgs::msg::Path::SharedPtr msg)
{
    local_path_ = msg;
}

void ControllerManagerNode::on_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_ = msg;
}

void ControllerManagerNode::on_can_callback(const race_msgs::msg::CAN::SharedPtr msg)
{
    current_rpm_ = msg->engine_rpm;
    current_gear_ = msg->selected_gear;
}

double ControllerManagerNode::calculate_heading_error()
{
    if (!local_path_ || !odom_ || local_path_->points.empty())
    {
        return 0.0;
    }

    const auto& target = local_path_->points.front().pose.position;
    const auto& pos = odom_->pose.pose.position;
    double yaw = tf2::getYaw(odom_->pose.pose.orientation);

    double dx = target.x - pos.x;
    double dy = target.y - pos.y;
    double target_yaw = std::atan2(dy, dx);

    double error = target_yaw - yaw;
    return normalize_angle(error);
}

double ControllerManagerNode::normalize_angle(double angle)
{
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

void ControllerManagerNode::runStep()
{
    if (!should_publish_control_ || vehicle_flag_ != race_msgs::msg::VehicleFlag::GREEN)
    {
        autoware_control_msgs::msg::Control control;
        control.longitudinal.acceleration = -3.0;
        control.longitudinal.is_defined_acceleration = true;
        control_pub_->publish(control);
        RCLCPP_DEBUG(this->get_logger(), "Braking because control is disabled or flag is not GREEN");
        return;
    }

    double heading_error = calculate_heading_error();

    RCLCPP_DEBUG(this->get_logger(),
                 "RPM: %.1f, Gear: %d, Heading error: %.3f rad (%.2f deg)",
                 current_rpm_, current_gear_,
                 heading_error, heading_error * 180.0 / M_PI);

    auto now = this->now();
    if (current_rpm_ >= rpm_threshold_ &&
        current_gear_ < max_gear_ &&
        std::abs(heading_error) < heading_tolerance_rad_ &&
        (now - last_shift_time_) > shift_cooldown_)
    {
        current_gear_ += 1;
        last_shift_time_ = now;
        RCLCPP_INFO(this->get_logger(), "Upshifted to gear %d", current_gear_);
    }

    autoware_control_msgs::msg::Control control;
    control.longitudinal.acceleration = 1.0;
    control.longitudinal.is_defined_acceleration = true;
    control.lateral.steering_tire_angle = 0.0;
    control.gear = current_gear_;
    control_pub_->publish(control);
}

void ControllerManagerNode::checkFlagTimeout()
{
    if ((this->now() - last_flag_time_).seconds() > flag_timeout_)
    {
        vehicle_flag_ = race_msgs::msg::VehicleFlag::RED;
        RCLCPP_WARN(this->get_logger(), "Vehicle flag timeout, set to RED");
    }
}

}  // namespace control

