#include <cstdio>
#include "global_planning/global_planner_manager.hpp"
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <cmath> // For sqrt and atan2 functions
#include "global_planning/global_planner_interface.hpp"
#include "global_planning/planners/race_planner.hpp"
#include "global_planning/planners/parking_planner.hpp"
#include "global_planning/planners/nav_planner.hpp"

namespace ROAR
{
    namespace GlobalPlanning
    {
        GlobalPlannerManager::GlobalPlannerManager() : nav2_util::LifecycleNode("global_planner_manager")
        {
            this->declare_parameter("debug", false);
            this->declare_parameter("planner_name", "RacePlanner");
            this->declare_parameter("planner_frequency", 0.1);

            std::string planner_name = this->get_parameter("planner_name").as_string();

            if (this->get_parameter("debug").as_bool())
            {
                auto logger_level_set = rcutils_logging_set_logger_level(get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
                logger_level_set = rcutils_logging_set_logger_level(planner_name.c_str(), RCUTILS_LOG_SEVERITY_DEBUG);         // enable or disable debug
                logger_level_set = rcutils_logging_set_logger_level("rclcpp", RCUTILS_LOG_SEVERITY_DEBUG);
                if (!logger_level_set)
                {
                    // Log error message
                    RCLCPP_ERROR(this->get_logger(), "Failed to set logger level to debug for planner: %s", planner_name.c_str());
                }
                else
                {
                    // Log success message
                    RCLCPP_INFO(this->get_logger(), "Logger level set to debug for planner: %s", planner_name.c_str());
                }
            }

            if (planner_name == "RacePlanner")
            {
                this->planner = new RacePlanner(this);
            }
            else if (planner_name == "ParkingPlanner")
            {
                this->planner = new ParkingPlanner(this);
            }
            else if (planner_name == "NavPlanner")
            {
                this->planner = new NavPlanner(this);
            }

            RCLCPP_INFO_STREAM(get_logger(), "Global Planner Manager has been initialized. Debug: " << this->get_parameter("debug").as_bool() << " Using Planner: " << planner_name);
        }

        GlobalPlannerManager::~GlobalPlannerManager()
        {
            // Destructor logic, if any
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is destroyed.");
        }
        nav2_util::CallbackReturn GlobalPlannerManager::on_configure(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now configured.");

            this->planner->initialize();

            // subscribers
            current_pose_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/roar/odometry",
                rclcpp::QoS(rclcpp::KeepLast(1)),
                [&](const nav_msgs::msg::Odometry::SharedPtr msg)
                {
                    this->current_odom = msg;
                });

            // publishers
            this->global_path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("global_path", 10);
            if (!this->planner->on_configure())
            {
                return nav2_util::CallbackReturn::FAILURE;
            }
            this->m_diagnostic_publisher = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1).reliable());

            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn GlobalPlannerManager::on_activate(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now activating.");
            this->timer = this->create_wall_timer(
                std::chrono::milliseconds((int)(this->get_parameter("planner_frequency").as_double() * 1000)),
                std::bind(&GlobalPlannerManager::step, this));

            this->global_path_publisher_->on_activate();

            if (!this->planner->on_activate())
            {
                return nav2_util::CallbackReturn::FAILURE;
            }
            this->m_diagnostic_publisher->on_activate();

            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn GlobalPlannerManager::on_deactivate(const rclcpp_lifecycle::State &state)
        {
            this->m_diagnostic_publisher->on_deactivate();
            this->global_path_publisher_->on_deactivate();
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now inactive.");
            if (!this->planner->on_deactivate())
            {
                return nav2_util::CallbackReturn::FAILURE;
            }

            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn GlobalPlannerManager::on_cleanup(const rclcpp_lifecycle::State &state)
        {

            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now cleaned up.");
            if (!this->planner->on_cleanup())
            {
                return nav2_util::CallbackReturn::FAILURE;
            }

            return nav2_util::CallbackReturn::SUCCESS;
        }

        nav2_util::CallbackReturn GlobalPlannerManager::on_shutdown(const rclcpp_lifecycle::State &state)
        {
            RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now shutting down.");
            if (!this->planner->on_shutdown())
            {
                RCLCPP_ERROR(get_logger(), "Failed to shutdown planner");
                return nav2_util::CallbackReturn::FAILURE;
            }

            return nav2_util::CallbackReturn::SUCCESS;
        }

        void GlobalPlannerManager::step()
        {
            // RCLCPP_INFO(get_logger(), "GlobalPlannerManager is now stepping.");
            if (this->planner == nullptr)
            {
                RCLCPP_ERROR(get_logger(), "Planner is not initialized");
                this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "GlobalPlannerManager", "Planner is not initialized");
                this->p_sendDiagMessage();
                return;
            }
            if (this->current_odom == nullptr)
            {
                // probably not initialized yet
                this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "GlobalPlannerManager", "Current odom is not initialized");
                this->p_sendDiagMessage();
                // RCLCPP_ERROR(get_logger(), "Current odom is not initialized");
                return;
            }
            // RCLCPP_DEBUG(get_logger(), "GlobalPlannerManager is now stepping.");
            this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "Global Planner Step", "GlobalPlannerManager is stepping");
            StepInput input;
            input.odom = this->current_odom;
            StepResult result = this->planner->step(input);
            this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "Global Planner Step", "Planner computed");

            // publish global path
            if (result.global_path != nullptr)
            {
                // RCLCPP_DEBUG(get_logger(), "Publishing global path");
                this->global_path_publisher_->publish(*result.global_path);
                this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "Global Planner Step", "Plan published");
            }
            else
            {
                // RCLCPP_DEBUG(get_logger(), "Global path is null, try selecting a path?");
                this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Global Planner Step", "Global path is null");
            }
            this->p_sendDiagMessage();
        }

    } // namespace global_planning
} // namespace roar

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ROAR::GlobalPlanning::GlobalPlannerManager>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}
