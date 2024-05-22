#include "global_planning/planners/hybrid_a_star/costmap_subscriber.h"

using std::placeholders::_1;

CostMapSubscriber::CostMapSubscriber(const std::string &topic_name, size_t buff_size)
    : Node("costmap_subscriber")
{
    subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        topic_name,
        buff_size,
        std::bind(&CostMapSubscriber::MessageCallBack, this, _1));
}

void CostMapSubscriber::MessageCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg_ptr)
{
    std::lock_guard<std::mutex> lock(buff_mutex_);
    deque_costmap_.emplace_back(costmap_msg_ptr);
}

void CostMapSubscriber::ParseData(std::deque<nav_msgs::msg::OccupancyGrid::SharedPtr> &deque_costmap_msg_ptr)
{
    std::lock_guard<std::mutex> lock(buff_mutex_);
    if (!deque_costmap_.empty())
    {
        deque_costmap_msg_ptr.insert(
            deque_costmap_msg_ptr.end(),
            deque_costmap_.begin(),
            deque_costmap_.end());

        deque_costmap_.clear();
    }
}
