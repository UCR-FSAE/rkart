#ifndef ROAR_CONTROL__PLUGIN__PURE_PURSUIT_LAT_CONTROL_HPP_
#define ROAR_CONTROL__PLUGIN__PURE_PURSUIT_LAT_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "controller_manager/controller_plugin_interface.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "roar_msgs/msg/vehicle_control.hpp"
#include "controller_manager/controller_state.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

using namespace roar::control;
namespace roar
{
    namespace control
    {
        struct PurePursuitLatControllerConfig
        {
            double wheel_base_m = 2.9;
            double look_ahead_distance = 1.0; // look_forward_distance
            double look_forward_gain = 0.1; // look forward gain
            double dt = 0.01;
        };

        struct PurePursuitLatControllerState 
        {
            rclcpp::Time prev_time = rclcpp::Time(0, 0);
            ControllerManagerState::SharedPtr latest_state = nullptr;
            double prev_steering_angle = 0.0;
        };

        class PurePursuitLatControllerPlugin : public ControllerPlugin
        {
            typedef std::shared_ptr<PurePursuitLatControllerPlugin> SharedPtr;
            typedef std::unique_ptr<PurePursuitLatControllerPlugin> UniquePtr;

            const char *get_plugin_name() override
            {
                return "PurePursuitLatController";
            }
            void initialize(nav2_util::LifecycleNode *node) override
            {
                ControllerPlugin::initialize(node); // Call the base class's initialize function
                config_ = PurePursuitLatControllerConfig{
                    this->node().declare_parameter<double>("pure_pursuit_controller.wheel_base_m", 2.9),
                    this->node().declare_parameter<double>("pure_pursuit_controller.look_ahead_distance", 1.0),
                    this->node().declare_parameter<double>("pure_pursuit_controller.look_forward_gain", 0.1),
                    this->node().declare_parameter<double>("pure_pursuit_controller.dt", 0.01),
                };

                // print config
                RCLCPP_INFO_STREAM(this->node().get_logger(), "[PurePursuitLatControllerPlugin] "
                                                                  << "\n"
                                                                  << " wheel_base_m: " << config_.wheel_base_m << "\n"
                                                                  << " look_ahead_distance: " << config_.look_ahead_distance << "\n"
                                                                  << " look_forward_gain: " << config_.look_forward_gain << "\n"
                                                                  << " dt: " << config_.dt);
            }

            bool configure(const ControllerManagerConfig::SharedPtr config) override
            {
                return true;
            }
            bool update(const ControllerManagerState::SharedPtr state) override
            {
                this->state_.latest_state = state;  // store the latest state
                return true;
            }
            bool compute(roar_msgs::msg::VehicleControl::SharedPtr controlMsg)
            {
                // find dt
                if (pCheckOrFillDt() == false)
                {
                    RCLCPP_WARN(node().get_logger(), "dt is 0, skipping this iteration");
                    return false;
                }

                auto time_now = node().now();

                if (time_now <= this->state_.prev_time)
                {
                    this->state_.prev_time = time_now;
                    RCLCPP_WARN(node().get_logger(), "this_pid_time is less than last_pid_time, skipping this iteration");
                    return false;
                }
                auto dt = (time_now - this->state_.prev_time).seconds();
                this->state_.prev_time = time_now;

                // check if sufficient dt has passed
                if (dt <= this->config_.dt)
                {
                    // RCLCPP_DEBUG(node().get_logger(), "dt is less than configured dt, using previous steering angle");
                    controlMsg->steering_angle = this->state_.prev_steering_angle;
                    return true;
                }

                if (this->state_.latest_state == nullptr)
                {
                    RCLCPP_WARN(node().get_logger(), "latest state is null, skipping this iteration");
                    return false;
                }
                if (this->state_.latest_state->vehicle_state == nullptr)
                {
                    RCLCPP_WARN(node().get_logger(), "latest_state->vehicle_state is null, skipping this iteration");
                    return false;
                }
                // find current x, y, yaw, velocity
                double x = 0.0;
                double y = 0.0;
                double speed = this->getSpeedFromOdom(this->state_.latest_state->vehicle_state->odometry);
                double yaw = this->getYawFromOdom(this->state_.latest_state->vehicle_state->odometry);

                double yaw_degree = yaw * 180 / M_PI;
                RCLCPP_DEBUG_STREAM(node().get_logger(), "yaw: " << yaw << " yaw_degree: " << yaw_degree);

                // find rear_x, rear_y
                double rear_x = x - (this->config_.wheel_base_m / 2) * cos(yaw);
                double rear_y = y - (this->config_.wheel_base_m / 2) * sin(yaw);

                // sanity check for path, finding the length of path should be > 0
                if (this->state_.latest_state->path_ego_centric.poses.size() == 0)
                {
                    RCLCPP_WARN(node().get_logger(), "path_ego_centric is empty, skipping this iteration");
                    return false;
                }
                // find the next waypoint
                int next_waypoint = p_findNextWaypoint(state_.latest_state->path_ego_centric);

                // get target_x and target_y, making these weird transformation because we are computing on a different frame
                double target_x = state_.latest_state->path_ego_centric.poses[next_waypoint].pose.position.y;
                double target_y = state_.latest_state->path_ego_centric.poses[next_waypoint].pose.position.x;

                RCLCPP_DEBUG_STREAM(node().get_logger(), "target_x: " << target_x << " target_y: " << target_y);

                // alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw
                double alpha = atan2(target_y - rear_y, target_x - rear_x) - yaw;
                RCLCPP_DEBUG_STREAM(node().get_logger(), "alpha: " << alpha);

                // delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)
                float Lf = this->config_.look_forward_gain * speed + this->config_.look_ahead_distance;
                double delta = atan2(2.0 * this->config_.wheel_base_m * sin(alpha) / Lf, 1.0);

                // convert rad to degree
                float delta_degree = delta * 180 / M_PI;

                RCLCPP_DEBUG_STREAM(node().get_logger(), "delta: " << delta << " delta_degree: " << delta_degree);

                // delta is steering angle
                controlMsg->steering_angle = delta_degree; // for some reason, our steering is reversed
                this->state_.prev_steering_angle = controlMsg->steering_angle;
                return true;
            }

            double getSpeedFromOdom(nav_msgs::msg::Odometry odom)
            {
                // square root of x^2 + y^2
                double x = odom.twist.twist.linear.x;
                double y = odom.twist.twist.linear.y;
                return sqrt(x * x + y * y);
            }
            double getYawFromOdom(nav_msgs::msg::Odometry odom)
            {
                // get yaw from quaternion
                auto quaternion = odom.pose.pose.orientation;
                tf2::Quaternion tf2_quat(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
                double roll, pitch, yaw;
                tf2::Matrix3x3(tf2_quat).getRPY(roll, pitch, yaw);

                // RCLCPP_DEBUG_STREAM(node().get_logger(), "roll: " << roll << " pitch: " << pitch << " yaw: " << yaw);
                return yaw; 
            }

            int p_findNextWaypoint(nav_msgs::msg::Path path)
            {
                int next_waypoint = 0;
                // RCLCPP_DEBUG_STREAM(node().get_logger(), "[PurePursuitLatControllerPlugin]: path size: " << path.poses.size());
                for (int i = 0; i < path.poses.size(); i++)
                {
                    float dist = sqrt(pow(path.poses[i].pose.position.x, 2) + pow(path.poses[i].pose.position.y, 2));
                    // RCLCPP_DEBUG_STREAM(node().get_logger(), "[PurePursuitLatControllerPlugin]: i:" << i << " dist: " << dist << " look_ahead_distance: " << config_.look_ahead_distance << " x: " << path.poses[i].pose.position.x << " y: " << path.poses[i].pose.position.y);
                    if (dist > config_.look_ahead_distance)
                    {
                        next_waypoint = i;
                        break;
                    }
                }
                float dist = sqrt(pow(path.poses[next_waypoint].pose.position.x, 2) + pow(path.poses[next_waypoint].pose.position.y, 2));

                // RCLCPP_DEBUG_STREAM(node().get_logger(), "[PurePursuitLatControllerPlugin]: next_waypoint index: " << next_waypoint << " dist: " << dist << " look_ahead_distance: " << config_.look_ahead_distance);
                return next_waypoint;
            }

            bool pCheckOrFillDt()
            {
                if (this->state_.prev_time.nanoseconds() == 0)
                {
                    this->state_.prev_time = node().now();
                    return false;
                }
                return true;
            }

        private:
            PurePursuitLatControllerConfig config_;
            PurePursuitLatControllerState state_;
            nav_msgs::msg::Path::SharedPtr path_;
        };
    } // namespace control
} // roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::control::PurePursuitLatControllerPlugin, roar::control::ControllerPlugin)

#endif // ROAR_CONTROL__PLUGIN__PURE_PURSUIT_LAT_CONTROL_HPP_