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
        class DummyLonControllerPlugin : public ControllerPlugin
        {
            typedef std::shared_ptr<DummyLonControllerPlugin> SharedPtr;
            typedef std::unique_ptr<DummyLonControllerPlugin> UniquePtr;

            const char *get_plugin_name() override
            {
                return "DummyLonControllerPlugin";
            }
            void initialize(nav2_util::LifecycleNode *node) override
            {
                ControllerPlugin::initialize(node); // Call the base class's initialize function
                this->target_speed_ = this->node().declare_parameter<double>("lon_control.target_speed", 5.0);
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
                controlMsg->target_speed = this->target_speed_;
                return true;
            }
        private:
            float target_speed_ = 5.0;
        };
    }
} // namespace roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::control::DummyLonControllerPlugin, roar::control::ControllerPlugin)

#endif // ROAR__CONTROL__PLUGIN__DUMMY_LON_CONTROLLER_HPP_