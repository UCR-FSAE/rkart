#ifndef ROAR__BEHAVIOR_PLANNING__CONDITION_NODES__IF_N_LAPS_REACHED_HPP_
#define ROAR__BEHAVIOR_PLANNING__CONDITION_NODES__IF_N_LAPS_REACHED_HPP_

#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/tree_node.h"
#include "behaviortree_cpp_v3/basic_types.h"
#include <nav_msgs/msg/odometry.hpp>
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"

#include <math.h>
namespace roar
{
    namespace planning
    {
        namespace behavior
        {
            namespace condition
            {
                class IfNLapsReached : public BT::ConditionNode
                {
                    struct IfNLapsReachedState 
                    {
                        bool has_entered = false;
                        bool has_exited = false;
                        int lap_counts = 0;

                        void reset_all()
                        {
                            reset_state();
                            lap_counts = 0;
                        }

                        void reset_state()
                        {
                            has_entered = false;
                            has_exited = false;
                        }
                    };
                    
                public:
                    IfNLapsReached(
                        const std::string &condition_name,
                        const BT::NodeConfiguration &conf,
                        const rclcpp::Logger &logger,
                        rclcpp::Clock &clock);
                    BT::NodeStatus tick() override;
                    static BT::PortsList providedPorts();

                    bool isWithinStartingRadius(const nav_msgs::msg::Odometry &odom, float radius, float x_init, float y_init)
                    {
                        float dx = odom.pose.pose.position.x - x_init;
                        float dy = odom.pose.pose.position.y - y_init;
                        float dist = std::sqrt(dx * dx + dy * dy);
                        return dist <= radius;
                    }

                private:
                    rclcpp::Logger logger_;
                    rclcpp::Clock &clock_;

                    int n_laps_ = 1;
                    float x_init_ = 0.0;
                    float y_init_ = 0.0;
                    float radius_ = 0.5;

                    IfNLapsReachedState state_;
                };

            } // namespace condition
        }     // namespace behavior
    }         // namespace planning

} // namespace roar

#endif // ROAR__BEHAVIOR_PLANNING__CONDITION_NODES__IF_N_LAPS_REACHED_HPP_