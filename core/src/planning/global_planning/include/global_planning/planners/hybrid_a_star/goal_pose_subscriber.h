#ifndef HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H
#define HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <deque>
#include <mutex>

class GoalPoseSubscriber2D : public rclcpp::Node
{
public:
    GoalPoseSubscriber2D(const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> &pose_data_buff);

private:
    void MessageCallBack(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose_ptr);

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscriber_;
    std::deque<geometry_msgs::msg::PoseStamped::SharedPtr> goal_poses_;

    std::mutex buff_mutex_;
};

#endif // HYBRID_A_STAR_GOAL_POSE_SUBSCRIBER_H