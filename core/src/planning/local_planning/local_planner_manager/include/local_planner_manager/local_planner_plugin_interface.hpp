#ifndef ROAR_PLANNING__PLUGIN_HPP_
#define ROAR_PLANNING__PLUGIN_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "local_planner_manager/local_planner_manager_state.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_listener.h>

namespace roar
{
    namespace planning
    {
        namespace local
        {
            class LocalPlannerPlugin
            {
            public:
                typedef std::shared_ptr<LocalPlannerPlugin> SharedPtr;
                typedef std::unique_ptr<LocalPlannerPlugin> UniquePtr;

                virtual ~LocalPlannerPlugin() = default;
                virtual void initialize(nav2_util::LifecycleNode *node) { 
                    node_ = node;
                    logger_ = rclcpp::get_logger(this->get_plugin_name());
                }
                virtual bool configure(const LocalPlannerManagerConfig::SharedPtr config) { return true; }
                virtual bool update(const LocalPlannerManagerState::SharedPtr state) { return true; }
                virtual nav_msgs::msg::Path::SharedPtr compute() = 0;
                virtual const char *get_plugin_name()
                {
                    return "LocalPlannerPlugin";
                }
                void setTfBuffer(std::shared_ptr<tf2_ros::Buffer> tf_buffer) { tf_buffer_ = tf_buffer; }
                std::shared_ptr<tf2_ros::Buffer> getTfBuffer() { return tf_buffer_; }

            protected:
                nav2_util::LifecycleNode &node() { return *node_; }
                rclcpp::Logger logger_{rclcpp::get_logger("LocalPlannerPlugin")};
                std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

            private:
                nav2_util::LifecycleNode *node_{};
                
                // logger
            };
        } // namespace local
    }     // namespace planning

} // namespace roar

#endif // ROAR_PLANNING__PLUGIN_HPP_