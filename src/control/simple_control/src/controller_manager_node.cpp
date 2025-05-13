#include "simple_control/controller_manager_node.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

namespace control
{

    ControllerManagerNode::ControllerManagerNode()
        : rclcpp::Node("controller_manager"),
          m_plugin_loader_("simple_control", "control::ControllerInterface")
    {
        this->declare_parameter("manager.debug", false);
        this->declare_parameter("manager.flag_timeout", 1.0);  // Timeout in seconds
        
        auto debug_ = this->get_parameter("manager.debug").as_bool();
        flag_timeout_ = this->get_parameter("manager.flag_timeout").as_double();
        
        if (debug_)
        {
            RCLCPP_INFO(this->get_logger(), "Debug mode enabled");
            rcutils_ret_t ret = rcutils_logging_set_logger_level(
                this->get_logger().get_name(),
                RCUTILS_LOG_SEVERITY_DEBUG);
            if (ret != RCUTILS_RET_OK)
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to set logger level to DEBUG");
            }
        }
        // Get parameters
        this->declare_parameter("manager.control_pub_rate", 10);
        const auto plugin_names = declare_parameter("manager.plugins", std::vector<std::string>{});
        // initialize plugins
        RCLCPP_INFO_STREAM(this->get_logger(), "plugin_names: " << plugin_names.size() << " plugins");
        for (const auto &plugin_name : plugin_names)
        {
            RCLCPP_DEBUG_STREAM(this->get_logger(), "plugin: " << plugin_name << " loading");

            control::ControllerInterface::SharedPtr new_plugin = m_plugin_loader_.createSharedInstance(plugin_name);
            m_plugins_.push_back(new_plugin);

            // debug setting for each plugin
            if (debug_) {
                rcutils_ret_t ret = rcutils_logging_set_logger_level(
                    new_plugin->get_plugin_name(),
                    RCUTILS_LOG_SEVERITY_DEBUG);
                if (ret != RCUTILS_RET_OK)
                {
                    RCLCPP_ERROR(this->get_logger(), "Failed to set logger level to DEBUG");
                }
                RCLCPP_DEBUG_STREAM(this->get_logger(), "plugin: " << new_plugin->get_plugin_name() << " logger level set to DEBUG");
            }
            RCLCPP_DEBUG_STREAM(this->get_logger(), "plugin: " << new_plugin->get_plugin_name() << " loaded");
        }
        RCLCPP_DEBUG_STREAM(this->get_logger(), "All (" << m_plugins_.size() << ") plugins loaded");
        std::for_each(
            m_plugins_.begin(), m_plugins_.end(), [this](control::ControllerInterface::SharedPtr &p)
            { 
            p->setup(this);
            RCLCPP_DEBUG_STREAM(this->get_logger(), "plugin: " << p->get_plugin_name() << " initialized"); });

        // Initialize publishers
        control_pub_ = this->create_publisher<autoware_control_msgs::msg::Control>(
            "control_cmd", 10);

        // Initialize subscribers
        should_publish_control_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "should_publish_control", 10,
            std::bind(&ControllerManagerNode::should_publish_control_callback, this, std::placeholders::_1));

        vehicle_flag_sub_ = this->create_subscription<race_msgs::msg::VehicleFlag>(
            "vehicle_flag", 10,
            std::bind(&ControllerManagerNode::vehicle_flag_callback, this, std::placeholders::_1));

        local_path_sub_ = this->create_subscription<autoware_planning_msgs::msg::Path>(
            "local_path", 10,
            std::bind(&ControllerManagerNode::on_local_path_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10,
            std::bind(&ControllerManagerNode::on_odom_callback, this, std::placeholders::_1));

        // Initialize control timer
        auto control_pub_rate_ = this->get_parameter("manager.control_pub_rate").as_int();
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / control_pub_rate_),
            std::bind(&ControllerManagerNode::runStep, this));
            
        // Initialize flag check timer (10Hz)
        flag_check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ControllerManagerNode::checkFlagTimeout, this));
            
        // Initialize last flag time
        last_flag_time_ = this->now();
        
        RCLCPP_INFO(this->get_logger(), "Flag timeout set to %.1f seconds", flag_timeout_);
    }

    void ControllerManagerNode::addController(const std::string &name, const std::string &type)
    {
        try
        {
            auto controller = m_plugin_loader_.createSharedInstance(type);
            controller->setup(this);
            m_plugins_.push_back(controller);
            RCLCPP_INFO(this->get_logger(), "Added controller: %s of type %s", name.c_str(), type.c_str());
        }
        catch (const pluginlib::PluginlibException &ex)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load controller: %s", ex.what());
        }
    }

    void ControllerManagerNode::should_publish_control_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        should_publish_control_ = msg->data;
    }

    void ControllerManagerNode::vehicle_flag_callback(const race_msgs::msg::VehicleFlag::SharedPtr msg)
    {
        vehicle_flag_ = msg->flag;
        last_flag_time_ = this->now();
        const char* flag_str = (msg->flag == race_msgs::msg::VehicleFlag::GREEN) ? "GREEN" :
                             (msg->flag == race_msgs::msg::VehicleFlag::RED) ? "RED" : "BLACK";
        RCLCPP_DEBUG(this->get_logger(), "Vehicle flag updated to: %s", flag_str);
    }

    void ControllerManagerNode::on_local_path_callback(const autoware_planning_msgs::msg::Path::SharedPtr msg)
    {
        local_path_ = msg;
    }

    void ControllerManagerNode::on_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = msg;
    }

    void ControllerManagerNode::runStep()
    {
        // Only run control when flag is GREEN
        if (!should_publish_control_ || vehicle_flag_ != race_msgs::msg::VehicleFlag::GREEN)
        {
            // publish brake control for RED or BLACK flag
            autoware_control_msgs::msg::Control control;
            control.longitudinal.acceleration = -3; // brake
            control.longitudinal.is_defined_acceleration = true;
            control.lateral.steering_tire_angle = 0.0;
            control_pub_->publish(control);
            
            const char* flag_str = (vehicle_flag_ == race_msgs::msg::VehicleFlag::RED) ? "RED" :
                                 (vehicle_flag_ == race_msgs::msg::VehicleFlag::BLACK) ? "BLACK" : "UNKNOWN";
            RCLCPP_DEBUG(this->get_logger(), "Braking due to vehicle flag = %s", flag_str);
            return;
        }
        
        State::SharedPtr state = std::make_shared<State>();
        generateCurrentState(state);

        autoware_control_msgs::msg::Control control;
        // Run each controller plugin
        for (auto &controller : m_plugins_)
        {
            try
            {
                controller->runStep(
                    state,
                    control);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(this->get_logger(), "Error running controller: %s", e.what());
            }
        }

        // Publish the control command
        control_pub_->publish(control);
    }

    void ControllerManagerNode::generateCurrentState(State::SharedPtr state)
    {
        // Get local path
        if (!local_path_)
        {
            // RCLCPP_DEBUG(this->get_logger(), "Unable to generate current state because local path is not available");
            return;
        }
        state->local_path = local_path_;

        // Get odom
        if (!odom_)
        {
            RCLCPP_DEBUG(this->get_logger(), "Unable to generate current state because odom is not available");
            return;
        }
        state->vehicle_state = std::make_shared<VehicleState>(VehicleState{
            .x = odom_->pose.pose.position.x,
            .y = odom_->pose.pose.position.y,
            .yaw = tf2::getYaw(tf2::Quaternion(odom_->pose.pose.orientation.x, odom_->pose.pose.orientation.y, odom_->pose.pose.orientation.z, odom_->pose.pose.orientation.w)),
            .velocity = odom_->twist.twist.linear.x
        });
    }

    void ControllerManagerNode::checkFlagTimeout()
    {
        auto current_time = this->now();
        double time_since_last_flag = (current_time - last_flag_time_).seconds();
        
        if (time_since_last_flag > flag_timeout_ && vehicle_flag_ != race_msgs::msg::VehicleFlag::RED)
        {
            RCLCPP_WARN(this->get_logger(), "No flag received for %.1f seconds, defaulting to RED", time_since_last_flag);
            vehicle_flag_ = race_msgs::msg::VehicleFlag::RED;
        }
    }
} // namespace control

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<control::ControllerManagerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}