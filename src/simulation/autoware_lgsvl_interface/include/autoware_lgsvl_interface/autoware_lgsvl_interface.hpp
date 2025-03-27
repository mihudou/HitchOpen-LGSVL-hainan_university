#ifndef AUTOWARE_LGSVL_INTERFACE__AUTOWARE_LGSVL_INTERFACE_HPP_
#define AUTOWARE_LGSVL_INTERFACE__AUTOWARE_LGSVL_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <lgsvl_msgs/msg/vehicle_control_data.hpp>
#include <autoware_control_msgs/msg/control.hpp>

namespace autoware_lgsvl_interface
{

class AutowareLgsvlInterface : public rclcpp::Node
{
public:
  explicit AutowareLgsvlInterface(const rclcpp::NodeOptions & options);

private:
  // Callback for processing incoming control messages
  void on_control_msg(const autoware_control_msgs::msg::Control::SharedPtr msg);

  // Publishers
  rclcpp::Publisher<lgsvl_msgs::msg::VehicleControlData>::SharedPtr m_lgsvl_control_pub;

  // Subscribers
  rclcpp::Subscription<autoware_control_msgs::msg::Control>::SharedPtr m_autoware_control_sub;


  float max_accel;
  float max_decel;
};

}  // namespace autoware_lgsvl_interface

#endif  // AUTOWARE_LGSVL_INTERFACE__AUTOWARE_LGSVL_INTERFACE_HPP_
