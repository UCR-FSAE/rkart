#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include <memory>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <boost/algorithm/string.hpp>

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#ifndef COSTMAP_NODE_MANAGER_HPP_
#define COSTMAP_NODE_MANAGER_HPP_

#define OBSTACLE 100
namespace costmap_node_manager
{
    struct CostmapNodeManagerConfig
    {
        typedef std::shared_ptr<CostmapNodeManagerConfig> SharedPtr;

        long int width = 100;
        long int height = 100;
        double resolution = 0.1; // meters per cell
        double loop_rate = 0.1;

        double inflation_radius = 0.5;
        double inflation_cost_scaling = 10.0;

        bool debug = true;
    };

    class CostmapNodeManager : public nav2_util::LifecycleNode
    {
    public:
        /**
         * @brief Constructor for roar_ros2_costmap_2d::ROS2Costmap2DNode
         */
        CostmapNodeManager();
        /**
         * @brief Destructor for roar_ros2_costmap_2d::ROS2Costmap2DNode
         */
        ~CostmapNodeManager();

    protected:
        /**
         * @brief Configures controller parameters and member variables
         *
         * Configures costmap;
         * @param state LifeCycle Node's state
         * @return Success or Failure
         * @throw pluginlib::PluginlibException When failed to initialize controller
         * plugin
         */
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Activates member variables
         *
         * Activates costmap
         * server
         * @param state LifeCycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Deactivates member variables
         *
         * Deactivates costmap
         * @param state LifeCycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Calls clean up states and resets member variables.
         *
         * Costmap clean up state is called, and resets rest of the
         * variables
         * @param state LifeCycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &state) override;
        /**
         * @brief Called when in Shutdown state
         * @param state LifeCycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &state) override;

        void updateOccupancyMap(std::shared_ptr<nav_msgs::msg::OccupancyGrid> occupancy_grid, std::vector<std::vector<double>> points);

        // laser scan subscriber
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
        sensor_msgs::msg::LaserScan::SharedPtr latest_laser_scan_;
        void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
        {
            if (this->latest_laser_to_base_link_transform != nullptr)
            {
                // assuming that transform between laser scan and base link does not change
                latest_laser_scan_ = msg;
                return;
            }

            // get the transform from laser scan to base link
            geometry_msgs::msg::TransformStamped transform;
            try
            {
                transform = this->tf_buffer_->lookupTransform("base_link", msg->header.frame_id, this->now());
                this->latest_laser_to_base_link_transform = std::make_shared<geometry_msgs::msg::TransformStamped>(transform);
                latest_laser_scan_ = msg;
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(get_logger(), "Transform error: %s", ex.what());
                return;
            }
        }

        // odom subscriber
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        nav_msgs::msg::Odometry::SharedPtr latest_odom_;
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            latest_odom_ = msg;
            // RCLCPP_DEBUG(get_logger(), "Received odometry");
        }

        // costmap publisher
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>> costmap_pub_;
        // footprint publisher
        std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PolygonStamped>> footprint_pub_;
        geometry_msgs::msg::PolygonStamped::SharedPtr latest_footprint_;
        void publish_footprint();
        

        // config
        CostmapNodeManagerConfig::SharedPtr m_config_;

        // callback timer
        rclcpp::TimerBase::SharedPtr execution_timer;
        void execution_callback();

        // tf
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        geometry_msgs::msg::TransformStamped::SharedPtr latest_laser_to_base_link_transform;
    };
}
#endif // COSTMAP_NODE_MANAGER_HPP_
