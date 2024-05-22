#ifndef ROAR__CONTROL__PLUGIN__DUMMY_LON_CONTROLLER_HPP_
#define ROAR__CONTROL__PLUGIN__DUMMY_LON_CONTROLLER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "controller_manager/controller_plugin_interface.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "roar_msgs/msg/vehicle_control.hpp"
#include "roar_msgs/msg/vehicle_state.hpp"
#include "controller_manager/controller_state.hpp"
using namespace roar::control;
namespace roar
{
    namespace control
    {
        struct SteeringSmootherConfig
        {
            bool debug_ = false;
            int buffer_len = 10;
        };
        class SteeringSmootherControllerPlugin : public ControllerPlugin
        {
            typedef std::shared_ptr<SteeringSmootherControllerPlugin> SharedPtr;
            typedef std::unique_ptr<SteeringSmootherControllerPlugin> UniquePtr;

            const char *get_plugin_name() override
            {
                return "SteeringSmootherController";
            }
            void initialize(nav2_util::LifecycleNode *node) override
            {
                ControllerPlugin::initialize(node); // Call the base class's initialize function
                this->config_ = SteeringSmootherConfig{
                    node->declare_parameter<bool>("steering_smoother.debug", false),
                    node->declare_parameter<int>("steering_smoother.buffer_len", 10)
                };
                if (this->config_.debug_)
                {
                    RCLCPP_INFO_STREAM(logger_, "Setting debug level for "
                                                    << this->get_plugin_name() << " to DEBUG");
                    bool _ = rcutils_logging_set_logger_level(this->get_plugin_name(),
                                                              RCUTILS_LOG_SEVERITY_DEBUG);
                }
                // print config
                RCLCPP_INFO_STREAM(logger_, "SteeringSmootherControllerPlugin config: "
                                                << "debug: " << this->config_.debug_
                                                << " buffer_len: " << this->config_.buffer_len);
            }

            bool configure(const ControllerManagerConfig::SharedPtr config)
            {
                return true;
            }

            bool update(const ControllerManagerState::SharedPtr state)
            {
                return true;
            }
            bool compute(roar_msgs::msg::VehicleControl::SharedPtr controlMsg)
            {
                RCLCPP_DEBUG_STREAM(logger_, "SteeringSmootherController::compute()");
                
                // if the time horizon is 0, return the control msg as is
                if (this->config_.buffer_len <= 0)
                {
                    return true;
                }

                // if the buffer is empty, return the control msg as is
                if (this->steering_buffer_.empty())
                {
                    steering_buffer_.push_back(*controlMsg);
                    return true;
                }

                // if buffer is full, pop the oldest element
                if (this->steering_buffer_.size() >= this->config_.buffer_len)
                {
                    this->steering_buffer_.erase(this->steering_buffer_.begin());
                }
                // push the new control msg
                this->steering_buffer_.push_back(*controlMsg);
                
                // compute the average of the steering angles
                double avg_steering_angle = 0;
                for (auto &steering_msg : this->steering_buffer_)
                {
                    avg_steering_angle += steering_msg.steering_angle;
                }
                avg_steering_angle /= this->steering_buffer_.size();
                RCLCPP_DEBUG_STREAM(logger_, "SteeringSmootherController::compute(): avg_steering_angle: " << avg_steering_angle << " latest_steering_angle: " << controlMsg->steering_angle);
                controlMsg->steering_angle = avg_steering_angle;
                return true;
            }

        private:
            SteeringSmootherConfig config_;
            
            // buffer for past n steering readings
            std::vector<roar_msgs::msg::VehicleControl> steering_buffer_;
        };
    }
} // namespace roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::control::SteeringSmootherControllerPlugin, roar::control::ControllerPlugin)

#endif // ROAR__CONTROL__PLUGIN__DUMMY_LON_CONTROLLER_HPP_