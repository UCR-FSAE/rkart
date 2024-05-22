#include "global_planning/planners/hybrid_a_star/init_pose_subscriber.h"

InitPoseSubscriber2D::InitPoseSubscriber2D(const std::string &node_name,
                                           const std::string &topic_name,
                                           size_t buff_size)
    : Node(node_name)
{
    subscriber_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        topic_name,
        buff_size,
        std::bind(&InitPoseSubscriber2D::MessageCallBack, this, std::placeholders::_1));
}

void InitPoseSubscriber2D::MessageCallBack(
    geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr init_pose_ptr)
{
    std::lock_guard<std::mutex> lock(buff_mutex_);
    init_poses_.emplace_back(init_pose_ptr);
}

void InitPoseSubscriber2D::ParseData(
    std::deque<geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr> &pose_data_buff)
{
    std::lock_guard<std::mutex> lock(buff_mutex_);
    if (!init_poses_.empty())
    {
        pose_data_buff.insert(pose_data_buff.end(), init_poses_.begin(), init_poses_.end());
        init_poses_.clear();
    }
}
