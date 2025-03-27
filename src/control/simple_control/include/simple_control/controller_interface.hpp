#ifndef SIMPLE_CONTROL__CONTROLLER_INTERFACE_HPP_
#define SIMPLE_CONTROL__CONTROLLER_INTERFACE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "autoware_control_msgs/msg/control.hpp"
#include "autoware_planning_msgs/msg/path.hpp"

namespace control {
    struct VehicleState {
        typedef std::shared_ptr<VehicleState> SharedPtr;
        double x;
        double y;
        double yaw;
        double velocity;
    };

    struct State {
        typedef std::shared_ptr<State> SharedPtr;
        VehicleState::SharedPtr vehicle_state;
        autoware_planning_msgs::msg::Path::SharedPtr local_path;
    };


    class ControllerInterface {
        public:
            typedef std::shared_ptr<ControllerInterface> SharedPtr;
            // Add default constructor
            ControllerInterface() 
                : logger_(rclcpp::get_logger("controller_interface")),
                  parent_node_(nullptr) {}
            
            virtual ~ControllerInterface() = default;

            virtual void setup(rclcpp::Node* node) {
                parent_node_ = node;
                logger_ = rclcpp::get_logger(this->get_plugin_name());
            }
            virtual void runStep(const State::SharedPtr state, autoware_control_msgs::msg::Control& control) = 0;
            
            virtual const char * get_plugin_name() = 0;
        protected:
            rclcpp::Logger logger_;
            rclcpp::Node* parent_node_;
    };
}
#endif
