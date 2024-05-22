#include "global_planning/planners/hybrid_a_star/goal_pose_subscriber.h"

GoalPoseSubscriber2D::GoalPoseSubscriber2D(const std::string &topic_name, size_t buff_size)
    : Node("goal_pose_subscriber_2d")
{
    subscriber_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        topic_name,
        rclcpp::QoS(rclcpp::KeepLast(buff_size)),
        std::bind(&GoalPoseSubscriber2D::MessageCallBack, this, std::placeholders::_1));
}

void GoalPoseSubscriber2D::MessageCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_ptr)
{
    std::lock_guard<std::mutex> lock(buff_mutex_);
    goal_poses_.emplace_back(goal_pose_ptr);
}

void GoalPoseSubscriber2D::ParseData(std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> &pose_data_buff)
{
    std::lock_guard<std::mutex> lock(buff_mutex_);
    if (!goal_poses_.empty())
    {
        pose_data_buff.insert(pose_data_buff.end(), goal_poses_.begin(), goal_poses_.end());
        goal_poses_.clear();
    }
}
