#include "global_planning/global_planning_node.hpp"
#include <fstream>
#include <sstream>
#include <iostream>
#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "rcutils/logging_macros.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

GlobalPlanningNode::GlobalPlanningNode() : Node("global_planning_node")
{
    this->declare_parameter("debug", false);
    this->declare_parameter("csv_file_path", "LVMS_SVL_ENU_TTL_15.csv");
    this->declare_parameter("global_path_publish_rate", 1);
    this->declare_parameter("local_path_publish_rate", 10);
    this->declare_parameter("local_path_length_m", 50.0);
    this->declare_parameter("planning_odom_publish_rate", 10);
    this->declare_parameter("visualize", false);

    auto debug_ = this->get_parameter("debug").as_bool();
    if (debug_) {
        RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
        rcutils_logging_set_logger_level(this->get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);
    }
    visualize_ = this->get_parameter("visualize").as_bool();
    csv_file_path_ = this->get_parameter("csv_file_path").as_string();
    local_path_length_m_ = this->get_parameter("local_path_length_m").as_double();

    path_pub_ = this->create_publisher<autoware_planning_msgs::msg::Path>("/planning/global_path", 1);
    local_path_pub_ = this->create_publisher<autoware_planning_msgs::msg::Path>("/planning/local_path", 1);
    planning_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/planning/odom", 1);
    if (visualize_) {
        global_path_vis_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/global_path_vis", 1);
        local_path_vis_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planning/local_path_vis", 1);
    }

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
        "/gps_top/fix", 10, std::bind(&GlobalPlanningNode::gpsCallback, this, std::placeholders::_1));
    lgsvl_odom_sub_ = this->create_subscription<lgsvl_msgs::msg::VehicleOdometry>(
        "/lgsvl/odometry", 10, std::bind(&GlobalPlanningNode::on_lgsvl_odom_callback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&GlobalPlanningNode::on_odom_callback, this, std::placeholders::_1));

    loadWaypointsFromCSV(csv_file_path_);

    int global_rate = this->get_parameter("global_path_publish_rate").as_int();
    global_path_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / global_rate),
        std::bind(&GlobalPlanningNode::publishGlobalPath, this));

    int local_rate = this->get_parameter("local_path_publish_rate").as_int();
    local_path_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / local_rate),
        std::bind(&GlobalPlanningNode::publishLocalPath, this));

    int odom_rate = this->get_parameter("planning_odom_publish_rate").as_int();
    planning_odom_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / odom_rate),
        std::bind(&GlobalPlanningNode::publishPlanningOdom, this));
}

void GlobalPlanningNode::publishGlobalPath()
{
    if (!received_gps_ || global_path_->points.empty()) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No GPS or waypoints available");
        return;
    }

    if (current_waypoint_index_ >= global_path_->points.size()) {
        current_waypoint_index_ = 0;
        RCLCPP_INFO(this->get_logger(), "Looping back to start of global path");
    }

    auto msg = autoware_planning_msgs::msg::Path();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.points.push_back(global_path_->points[current_waypoint_index_]);
    current_waypoint_index_++;

    path_pub_->publish(msg);

    if (visualize_) {
        auto vis = nav_msgs::msg::Path();
        convertAutowarePathToNavPath(msg, vis);
        global_path_vis_pub_->publish(vis);
    }
}

void GlobalPlanningNode::publishLocalPath()
{
    if (!received_gps_ || !current_position_ || global_path_->points.empty()) return;

    size_t next_idx = findNextWaypoint(current_position_, local_path_length_m_);
    auto msg = autoware_planning_msgs::msg::Path();
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";

    if (current_waypoint_index_ <= next_idx) {
        msg.points.insert(msg.points.end(),
                          global_path_->points.begin() + current_waypoint_index_,
                          global_path_->points.begin() + next_idx + 1);
    } else {
        msg.points.insert(msg.points.end(),
                          global_path_->points.begin() + current_waypoint_index_,
                          global_path_->points.end());
        msg.points.insert(msg.points.end(),
                          global_path_->points.begin(),
                          global_path_->points.begin() + next_idx + 1);
    }

    local_path_pub_->publish(msg);

    if (visualize_) {
        auto vis = nav_msgs::msg::Path();
        convertAutowarePathToNavPath(msg, vis);
        local_path_vis_pub_->publish(vis);
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GlobalPlanningNode>());
    rclcpp::shutdown();
    return 0;
}

