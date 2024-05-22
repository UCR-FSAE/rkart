/*******************************************************************************
 * Software License Agreement (BSD License)
 * Copyright (c) 2022 Zhang Zhimeng
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY
 * WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#ifndef HYBRID_A_STAR_DISPLAY_TOOLS_H
#define HYBRID_A_STAR_DISPLAY_TOOLS_H

#include "type.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

__attribute__((unused)) static void PublishSearchedTree(const TypeVectorVecd<4> &tree, const std::string &topic_name)
{
    static auto node = rclcpp::Node::make_shared("display_tools_node");
    static auto tree_pub = node->create_publisher<visualization_msgs::msg::Marker>(topic_name, 10);

    visualization_msgs::msg::Marker tree_list;
    tree_list.header.frame_id = "world";
    tree_list.header.stamp = node->get_clock()->now();
    tree_list.type = visualization_msgs::msg::Marker::LINE_LIST;
    tree_list.action = visualization_msgs::msg::Marker::ADD;
    tree_list.ns = "searched_tree";
    tree_list.scale.x = 0.02;

    tree_list.color.a = 0.8;
    tree_list.color.r = 136;
    tree_list.color.g = 138;
    tree_list.color.b = 133;

    tree_list.pose.orientation.w = 1.0;
    tree_list.pose.orientation.x = 0.0;
    tree_list.pose.orientation.y = 0.0;
    tree_list.pose.orientation.z = 0.0;

    geometry_msgs::msg::Point point;
    for (const auto &i : tree)
    {
        point.x = i.x();
        point.y = i.y();
        point.z = 0.0;
        tree_list.points.push_back(point);

        point.x = i.z();
        point.y = i.w();
        point.z = 0.0;
        tree_list.points.push_back(point);
    }

    tree_pub->publish(tree_list);
}

__attribute__((unused)) static void PublishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr &path_pub, const TypeVectorVecd<3> &path)
{
    nav_msgs::msg::Path nav_path;

    geometry_msgs::msg::PoseStamped pose_stamped;
    for (const auto &pose : path)
    {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, pose.z());
        pose_stamped.pose.orientation = tf2::toMsg(quat);

        nav_path.poses.push_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = rclcpp::Node::make_shared("display_tools_node")->get_clock()->now();

    path_pub->publish(nav_path);
}

__attribute__((unused)) static void PublishPath(const TypeVectorVecd<3> &path, const std::string &topic_name)
{
    static auto node = rclcpp::Node::make_shared("display_tools_node");
    static auto path_pub = node->create_publisher<nav_msgs::msg::Path>(topic_name, 1);

    nav_msgs::msg::Path nav_path;

    geometry_msgs::msg::PoseStamped pose_stamped;
    for (const auto &pose : path)
    {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;

        tf2::Quaternion quat;
        quat.setRPY(0, 0, pose.z());
        pose_stamped.pose.orientation = tf2::toMsg(quat);

        nav_path.poses.push_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = node->get_clock()->now();

    path_pub->publish(nav_path);
}

__attribute__((unused)) static void PublishPath(const TypeVectorVecd<2> &path, const std::string &topic_name)
{
    static auto node = rclcpp::Node::make_shared("display_tools_node");
    static auto path_pub = node->create_publisher<nav_msgs::msg::Path>(topic_name, 10);

    nav_msgs::msg::Path nav_path;

    geometry_msgs::msg::PoseStamped pose_stamped;
    for (const auto &pose : path)
    {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;

        nav_path.poses.push_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = node->get_clock()->now();

    path_pub->publish(nav_path);
}

__attribute__((unused)) static void PublishEllipse(const Vec2d &x_center, double &c_best, double dist,
                                                   double theta, const std::string &topic_name)
{
    double a = std::sqrt(c_best * c_best - dist * dist) * 0.5;
    double b = c_best * 0.5;
    double angle = M_PI / 2 - theta;

    Eigen::Matrix3d R_z = Eigen::AngleAxisd(-angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    TypeVectorVecd<2> ellipse_path;

    Eigen::Vector2d coord;
    for (double t = 0; t <= 2.0 * M_PI + 0.1; t += 0.1)
    {
        coord.x() = a * std::cos(t);
        coord.y() = b * std::sin(t);

        coord = R_z.block<2, 2>(0, 0) * coord + x_center;
        ellipse_path.emplace_back(coord);
    }

    static auto node = rclcpp::Node::make_shared("display_tools_node");
    static auto path_pub = node->create_publisher<nav_msgs::msg::Path>(topic_name, 10);

    nav_msgs::msg::Path nav_path;

    geometry_msgs::msg::PoseStamped pose_stamped;
    for (const auto &pose : ellipse_path)
    {
        pose_stamped.header.frame_id = "world";
        pose_stamped.pose.position.x = pose.x();
        pose_stamped.pose.position.y = pose.y();
        pose_stamped.pose.position.z = 0.0;

        nav_path.poses.push_back(pose_stamped);
    }

    nav_path.header.frame_id = "world";
    nav_path.header.stamp = node->get_clock()->now();

    path_pub->publish(nav_path);
}

#endif // HYBRID_A_STAR_DISPLAY_TOOLS_H
