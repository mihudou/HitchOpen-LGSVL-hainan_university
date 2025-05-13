#include "autoware_lgsvl_interface/autoware_lgsvl_interface.hpp"

namespace autoware_lgsvl_interface
{

AutowareLgsvlInterface::AutowareLgsvlInterface(
  const rclcpp::NodeOptions & options)
: Node("autoware_lgsvl_interface", options)
{
  this->declare_parameter("max_accel_mps2", 3.0f);
  this->declare_parameter("max_decel_mps2", -3.0f);

  max_accel = this->get_parameter("max_accel_mps2").as_double();
  max_decel = this->get_parameter("max_decel_mps2").as_double();

  // Publishers
  m_lgsvl_control_pub = create_publisher<lgsvl_msgs::msg::VehicleControlData>(
    "vehicle_control_cmd", rclcpp::QoS{10});

  // Subscribers
  m_autoware_control_sub = create_subscription<autoware_control_msgs::msg::Control>(
    "control_cmd", rclcpp::QoS{10},
    std::bind(&AutowareLgsvlInterface::on_control_msg, this, std::placeholders::_1));
}

void AutowareLgsvlInterface::on_control_msg(
  const autoware_control_msgs::msg::Control::SharedPtr msg)
{
  auto lgsvl_msg = lgsvl_msgs::msg::VehicleControlData();
  
  // Set header timestamp
  lgsvl_msg.header.stamp = msg->stamp;
  
  // Convert longitudinal control
  // For acceleration, positive values go to acceleration_pct, negative to braking_pct
  if (msg->longitudinal.acceleration >= 0.0f) {
    // Map acceleration to 0-1 range. You may need to adjust the scaling factor
    lgsvl_msg.acceleration_pct = std::min(msg->longitudinal.acceleration / max_accel, 1.0f);
    lgsvl_msg.braking_pct = 0.0f;
  } else {
    // Map deceleration to braking
    lgsvl_msg.acceleration_pct = 0.0f;
    lgsvl_msg.braking_pct = std::abs(std::min(std::abs(msg->longitudinal.acceleration) / max_decel, 1.0f));
  }

  // Convert lateral control
  lgsvl_msg.target_wheel_angle = msg->lateral.steering_tire_angle;

  // Set gear based on acceleration direction
  if (std::abs(msg->longitudinal.acceleration) < 0.01f || max_accel == 0.0f) {
    lgsvl_msg.target_gear = lgsvl_msg.GEAR_NEUTRAL;
  } else if (msg->longitudinal.acceleration > 0.0f) {
    lgsvl_msg.target_gear = lgsvl_msg.GEAR_DRIVE;
  } else {
    lgsvl_msg.target_gear = lgsvl_msg.GEAR_NEUTRAL; // brake, no reverse gear
  }


  // Publish the converted message
  m_lgsvl_control_pub->publish(lgsvl_msg);
}

}  // namespace autoware_lgsvl_interface

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options{};
  auto node = std::make_shared<autoware_lgsvl_interface::AutowareLgsvlInterface>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

