#ifndef NAV_PLANNER_INTERFACE_HPP
#define NAV_PLANNER_INTERFACE_HPP

#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <map>
#include <boost/any.hpp>
#include <math.h>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace ROAR {
    namespace GlobalPlanning
    {
            struct NavPlannerInputs {
                using SharedPtr = std::shared_ptr<NavPlannerInputs>; // Define SharedPtr alias

                geometry_msgs::msg::Pose::SharedPtr start;
                geometry_msgs::msg::Pose::SharedPtr goal;
                nav_msgs::msg::OccupancyGrid::SharedPtr map;
            };
            
            class NavPlannerInterface {

                public:
                    virtual bool init(rclcpp_lifecycle::LifecycleNode *parent)
                    {
                        m_node_ = parent;
                        return true;
                    }
                    virtual bool configure(const std::map<std::string, boost::any> configuration) = 0;
                    virtual bool setStart(std::shared_ptr<geometry_msgs::msg::Pose> start) = 0;
                    virtual bool setGoal(std::shared_ptr<geometry_msgs::msg::Pose> goal) = 0;
                    virtual bool setMap(nav_msgs::msg::OccupancyGrid::SharedPtr map) = 0;

                    virtual bool setInputs(std::shared_ptr<NavPlannerInputs> inputs) = 0;

                    // given the input, the planner should return a path
                    virtual nav_msgs::msg::Path::SharedPtr plan() = 0;
                    virtual void reset() = 0;
                protected:
                    rclcpp_lifecycle::LifecycleNode *m_node_{};
                    std::shared_ptr<NavPlannerInputs> m_inputs;
            };
    } // namespace GlobalPlanning
} // namespace ROAR

#endif //NAV_PLANNER_INTERFACE_HPP