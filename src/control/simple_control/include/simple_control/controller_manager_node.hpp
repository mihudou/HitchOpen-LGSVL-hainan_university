#ifndef SIMPLE_CONTROL__CONTROLLER_MANAGER_HPP_
#define SIMPLE_CONTROL__CONTROLLER_MANAGER_HPP_

#include "simple_control/controller_interface.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "std_msgs/msg/bool.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_planning_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
namespace control {

    class ControllerManagerNode: public rclcpp::Node {
        public:
            ControllerManagerNode();
            void addController(const std::string& name, const std::string& type);
            void runStep();

            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr should_publish_control_sub_;
            void should_publish_control_callback(const std_msgs::msg::Bool::SharedPtr msg);
            bool should_publish_control_ = true;


            rclcpp::Publisher<autoware_control_msgs::msg::Control>::SharedPtr control_pub_; 
            rclcpp::TimerBase::SharedPtr control_timer_;

            rclcpp::Subscription<autoware_planning_msgs::msg::Path>::SharedPtr local_path_sub_;
            void on_local_path_callback(const autoware_planning_msgs::msg::Path::SharedPtr msg);
            autoware_planning_msgs::msg::Path::SharedPtr local_path_;
            
            typedef std::vector<control::ControllerInterface::SharedPtr> PluginList;
            pluginlib::ClassLoader<control::ControllerInterface> m_plugin_loader_;
            PluginList m_plugins_;

            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            void on_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
            nav_msgs::msg::Odometry::SharedPtr odom_;
        private:
            void generateCurrentState(State::SharedPtr state);
    };
}
#endif
