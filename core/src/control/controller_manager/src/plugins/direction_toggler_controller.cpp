#ifndef ROAR_CONTROL__PLUGIN__LONG_PID_CONTROL_HPP_
#define ROAR_CONTROL__PLUGIN__LONG_PID_CONTROL_HPP_

#include "rclcpp/rclcpp.hpp"
#include "controller_manager/controller_plugin_interface.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "roar_msgs/msg/vehicle_control.hpp"
#include "controller_manager/controller_state.hpp"
#include "controller_manager/pid_controller.hpp"

using namespace roar::control;
namespace roar
{
    namespace control
    {
        struct DirectionTogglerState
        {
            bool is_forward = true;
        };

        struct DirectionTogglerConfig
        {
            float safe_speed = 1.0; // m/s speed to allow switching forward/backward
        };

        struct WaypointDirectionDecision
        {
            bool is_forward;
            bool is_error;
        };

        class DirectionTogglerControllerPlugin : public ControllerPlugin
        {
            typedef std::shared_ptr<DirectionTogglerControllerPlugin> SharedPtr;
            typedef std::unique_ptr<DirectionTogglerControllerPlugin> UniquePtr;

            const char *get_plugin_name() override
            {
                return "DirectionTogglerControllerPlugin";
            }
            void initialize(nav2_util::LifecycleNode *node) override
            {
                ControllerPlugin::initialize(node); // Call the base class's initialize function
            }

            bool configure(const ControllerManagerConfig::SharedPtr config)
            {
                return true;
            }
            bool update(const ControllerManagerState::SharedPtr state)
            {
                controller_manager_state_ptr_ = state;
                return true;
            }
            bool compute(roar_msgs::msg::VehicleControl::SharedPtr controlMsg)
            {
                if (controller_manager_state_ptr_ == nullptr) {
                    // do not do anything if controller manager failed to run update step
                    return true;
                }

                // check if waypoint is ahead the car
                auto waypoint_direction_decision = isWaypointAheadOfVehicle(get_controller_manager_state_ptr_());
                bool is_waypoint_ahead = waypoint_direction_decision.is_forward;

                if (waypoint_direction_decision.is_error)
                {
                    RCLCPP_ERROR_STREAM(node().get_logger(), "Error in DirectionTogglerControllerPlugin::compute");
                    return true;
                }

                bool is_safe_to_switch = isVehicleSafeToSwitchState(get_controller_manager_state_ptr_());

                bool need_to_switch_state = is_waypoint_ahead != get_state_().is_forward;

                if (need_to_switch_state)
                {
                    // we need to switch state
                    if (is_safe_to_switch)
                    {
                        // it is safe to switch state
                        get_state_().is_forward = is_waypoint_ahead;
                        controlMsg->reverse = !get_state_().is_forward;
                        return true;
                    }
                    else
                    {
                        // it is not safe to switch state
                        RCLCPP_INFO_STREAM(node().get_logger(), "DirectionTogglerControllerPlugin::compute: vehicle need to switch state but is not safe to switch , slowing vehicle down");

                        controlMsg->target_speed = 0.0; // come to a gradual stop
                        controlMsg->steering_angle = 0.0;
                        return true;
                    }
                }
                RCLCPP_DEBUG_STREAM(node().get_logger(), "DirectionTogglerControllerPlugin::compute: vehicle does not need to switch state");
                return true;
            }

            WaypointDirectionDecision isWaypointAheadOfVehicle(const ControllerManagerState::SharedPtr state)
            {
                WaypointDirectionDecision decision;
                if (state->path_ego_centric.poses.size() == 0)
                {
                    decision.is_error = true;
                    return decision;
                }

                // get the last waypoint
                auto last_waypoint = state->path_ego_centric.poses.back().pose.position;

                if (last_waypoint.x < 0)
                {
                    decision.is_forward = false;
                }
                else
                {
                    decision.is_forward = true;
                }
                decision.is_error = false;
                return decision;
            }

            bool isVehicleSafeToSwitchState(const ControllerManagerState::SharedPtr state)
            {
                if (state->vehicle_state == nullptr)
                {
                    return false;
                }

                float currentVehicleSpeed = state->vehicle_state->vehicle_status.speed;
                if (currentVehicleSpeed < config_.safe_speed)
                {
                    return true;
                }
                return false;
            }

        private:
            DirectionTogglerState state_;
            DirectionTogglerConfig config_;

            ControllerManagerState::SharedPtr controller_manager_state_ptr_;

        protected: 
            DirectionTogglerState &get_state_()
            {
                return state_;
            }

            DirectionTogglerConfig &get_config_()
            {
                return config_;
            }

            ControllerManagerState::SharedPtr &get_controller_manager_state_ptr_()
            {
                return controller_manager_state_ptr_;
            }
        };
    } // namespace control
} // roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::control::DirectionTogglerControllerPlugin, roar::control::ControllerPlugin)

#endif // ROAR_CONTROL__PLUGIN__LONG_PID_CONTROL_HPP_