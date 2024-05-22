#ifndef ROAR_CONTROL__PLUGIN_HPP_
#define ROAR_CONTROL__PLUGIN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "controller_manager/controller_state.hpp"
#include "roar_msgs/msg/vehicle_control.hpp"

namespace roar
{
    namespace control
    {
        class ControllerPlugin
        {

        public:
            typedef std::shared_ptr<ControllerPlugin> SharedPtr;
            typedef std::unique_ptr<ControllerPlugin> UniquePtr;

            virtual ~ControllerPlugin() = default;
            virtual void initialize(nav2_util::LifecycleNode *node) { 
                node_ = node; 
                logger_ = rclcpp::get_logger(this->get_plugin_name());
            }
            virtual bool configure(const ControllerManagerConfig::SharedPtr config) { return true; }
            virtual bool update(const ControllerManagerState::SharedPtr state) { return true; }
            virtual bool compute(roar_msgs::msg::VehicleControl::SharedPtr controlMsg) = 0;
            virtual const char *get_plugin_name()
            {
                return "default plugin name";
            }

        protected:
            nav2_util::LifecycleNode &node() { return *node_; }
            rclcpp::Logger logger_{rclcpp::get_logger("LocalPlannerPlugin")};

        private:
            nav2_util::LifecycleNode *node_{};
        };

    } // namespace control
}
#endif // ROAR_CONTROL__PLUGIN_HPP_