#ifndef ROAR__GLOBAL_PLANNING__HYBRID_A_STAR_HPP
#define ROAR__GLOBAL_PLANNING__HYBRID_A_STAR_HPP


#include "global_planning/planners/nav_planner_interface.hpp"
#include "global_planning/planners/hybrid_a_star/hybrid_a_star.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>

namespace ROAR
{
    namespace GlobalPlanning
    {
        struct HybridAStarAlgoParameters
        {
            typedef std::shared_ptr<HybridAStarAlgoParameters> SharedPtr;

            double steering_angle;
            int steering_angle_discrete_num;
            double segment_length;
            int segment_length_discrete_num;
            double wheel_base;
            double steering_penalty;
            double reversing_penalty;
            double steering_change_penalty;
            double shot_distance;
            int grid_size_phi = 72;
        };

        class HybridAStarAlgo : public NavPlannerInterface
        {
        public:
            HybridAStarAlgo();
            ~HybridAStarAlgo();

            bool init(rclcpp_lifecycle::LifecycleNode *parent) override;
            bool configure(const std::map<std::string, boost::any> configuration) override;
            bool setStart(std::shared_ptr<geometry_msgs::msg::Pose> start) override;
            bool setGoal(std::shared_ptr<geometry_msgs::msg::Pose> goal) override;
            bool setMap(nav_msgs::msg::OccupancyGrid::SharedPtr map) override;

            bool setInputs(std::shared_ptr<NavPlannerInputs> inputs) override;

            nav_msgs::msg::Path::SharedPtr plan() override;
            void reset() override;
            double getYawFromQuaternion(const geometry_msgs::msg::Quaternion &msg)
            {
                tf2::Quaternion q(
                    msg.x,
                    msg.y,
                    msg.z,
                    msg.w);
                tf2::Matrix3x3 m(q);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                return yaw;
            }

            HybridAStarAlgoParameters::SharedPtr getParameters() const
            {
                return m_parameters;
            }

            void setParameters(HybridAStarAlgoParameters::SharedPtr parameter)
            {
                m_parameters = parameter;
            }

        protected:
            std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;
            std::shared_ptr<HybridAStarAlgoParameters> m_parameters{};
        };

    } // namespace GlobalPlanning
} // namespace ROAR

#endif //ROAR__GLOBAL_PLANNING__HYBRID_A_STAR_HPP