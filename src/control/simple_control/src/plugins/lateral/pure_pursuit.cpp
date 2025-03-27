#include <simple_control/controller_interface.hpp>
#include <autoware_planning_msgs/msg/path.hpp>
#include <boost/any.hpp>
#include <math.h>

namespace control
{
    namespace lateral
    {
        class PurePursuitController : public ControllerInterface
        {
        public:
            PurePursuitController() = default;

            void setup(rclcpp::Node *node) override
            {
                ControllerInterface::setup(node);
                // Get parameters
                look_ahead_distance_ = node->declare_parameter("lateral.pure_pursuit.look_ahead_distance", 15.0);
                wheel_base_ = node->declare_parameter("lateral.pure_pursuit.wheel_base", 2.7);
                max_steering_angle_ = node->declare_parameter("lateral.pure_pursuit.max_steering_angle", 0.52);
                smoothing_factor_ = node->declare_parameter("lateral.pure_pursuit.smoothing_factor", 0.5);
                previous_steering_ = 0.0;
            }

            void runStep(const State::SharedPtr state,
                         autoware_control_msgs::msg::Control &control) override
            {
                try
                {
                    if (!state->vehicle_state)
                    {
                        // RCLCPP_DEBUG(logger_, "Unable to run pure pursuit controller because vehicle state is not available");
                        return;
                    }
                    if (!state->local_path)
                    {
                        // RCLCPP_DEBUG(logger_, "Unable to run pure pursuit controller because local path is not available");
                        return;
                    }
                    const auto vehicle_state = state->vehicle_state;
                    const auto path = state->local_path;

                    // Find look-ahead point
                    auto target_point = findLookAheadPoint(vehicle_state, path);

                    // Calculate steering angle using pure pursuit formula
                    double alpha =  atan2(target_point.y - vehicle_state->y, target_point.x - vehicle_state->x) - vehicle_state->yaw;
                    double alpha_normalized = atan2(sin(alpha), cos(alpha));
                    double steering = atan2(2.0 * wheel_base_ * sin(alpha_normalized),
                                                 look_ahead_distance_);

                    // Limit steering angle
                    steering = std::clamp(steering, -max_steering_angle_, max_steering_angle_) * -1;

                    // Apply smoothing
                    steering = smoothing_factor_ * steering + (1.0 - smoothing_factor_) * previous_steering_;
                    previous_steering_ = steering;

                    RCLCPP_DEBUG_STREAM(logger_, "vehicle_state (x, y, yaw): (" << vehicle_state->x << ", " << vehicle_state->y << ", " << vehicle_state->yaw << ")");
                    RCLCPP_DEBUG_STREAM(logger_, "target_point (x, y): (" << target_point.x << ", " << target_point.y << ")");
                    RCLCPP_DEBUG_STREAM(logger_, "steering: " << (steering * 180.0 / M_PI) << " deg, alpha_normalized: " << alpha_normalized << "\n");

                    // Apply steering command
                    control.lateral.steering_tire_angle = steering;
                    control.lateral.is_defined_steering_tire_rotation_rate = false;
                }
                catch (const boost::bad_any_cast &e)
                {
                    RCLCPP_ERROR(logger_, "Error casting state variables: %s", e.what());
                }
            }

            const char *get_plugin_name() override
            {
                return "pure_pursuit";
            }

        private:
            double look_ahead_distance_;
            double wheel_base_;
            double max_steering_angle_;
            double smoothing_factor_;
            double previous_steering_;

            struct Point2D
            {
                double x;
                double y;
            };

            Point2D findLookAheadPoint(const VehicleState::SharedPtr vehicle,
                                       const autoware_planning_msgs::msg::Path::SharedPtr path)
            {
                // Point2D closest_point{path->points[0].pose.position.x,
                //                       path->points[0].pose.position.y};
                // double min_distance = std::numeric_limits<double>::max();

                // // Find the first point that is at least look_ahead_distance_ away
                // for (const auto &point : path->points)
                // {
                //     double dx = point.pose.position.x - vehicle->x;
                //     double dy = point.pose.position.y - vehicle->y;
                //     double distance = std::sqrt(dx * dx + dy * dy);

                //     if (distance >= look_ahead_distance_)
                //     {
                //         RCLCPP_DEBUG_STREAM(logger_, "look-ahead point: " << point.pose.position.x << ", " << point.pose.position.y << " distance: " << distance);
                //         return Point2D{point.pose.position.x, point.pose.position.y};
                //     }
                // }

                // If no point is found, return the last point
                return Point2D{path->points.back().pose.position.x,
                               path->points.back().pose.position.y};
            }
        };

    } // namespace lateral
} // namespace control

PLUGINLIB_EXPORT_CLASS(control::lateral::PurePursuitController, control::ControllerInterface)
