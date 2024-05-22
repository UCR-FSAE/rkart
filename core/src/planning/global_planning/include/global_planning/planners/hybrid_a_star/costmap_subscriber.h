#ifndef HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H
#define HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H

#include <rclcpp/rclcpp.hpp>
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <deque>
#include <mutex>
#include <thread>
#include <string>

class CostMapSubscriber : public rclcpp::Node
{
public:
    CostMapSubscriber(const std::string &topic_name, size_t buff_size);

    void ParseData(std::deque<nav_msgs::msg::OccupancyGrid::SharedPtr> &deque_costmap_msg_ptr);

private:
    void MessageCallBack(const nav_msgs::msg::OccupancyGrid::SharedPtr costmap_msg_ptr);

private:
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscriber_;
    std::deque<nav_msgs::msg::OccupancyGrid::SharedPtr> deque_costmap_;

    std::mutex buff_mutex_;
};

#endif // HYBRID_A_STAR_COSTMAP_SUBSCRIBER_H
