#include "global_planning/global_planner_interface.hpp"
#include "global_planning/planners/nav_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/buffer_core.h"
#include "global_planning/planners/hybrid_a_star/hybrid_a_star_algo.hpp"

namespace ROAR
{
    namespace GlobalPlanning
    {
        NavPlanner::NavPlanner(nav2_util::LifecycleNode *node) : GlobalPlannerInterface(node, "NavPlanner")
        {
            m_node_ = node;

            // config
            m_config_ = std::make_shared<NavPlannerConfig>(
                NavPlannerConfig{
                    m_node_->declare_parameter<std::string>("nav_planner.algo_name", "HybridAStarAlgo"),
                    m_node_->declare_parameter<bool>("nav_planner.debug", false),
                });

            // algo
            if (m_config_->algo_name == "HybridAStarAlgo")
            {
                m_nav_planner_algo = std::make_unique<HybridAStarAlgo>();
            }
            else
            {
                RCLCPP_ERROR(m_logger_, "Algo name not recognized, defaulting to HybridAStarAlgo");
                m_nav_planner_algo = std::make_unique<HybridAStarAlgo>();
            }
            bool status = m_nav_planner_algo->init(node);
            if (!status)
            {
                RCLCPP_ERROR(m_logger_, "Failed to initialize nav planner algo");
                return;
            }
            RCLCPP_INFO(m_logger_, "NavPlanner is initialized");
        }

        NavPlanner::~NavPlanner()
        {
        }

        void NavPlanner::initialize()
        {
            RCLCPP_INFO(m_logger_, "NavPlanner is initialized");
            tf_buffer_ =
                std::make_unique<tf2_ros::Buffer>(this->m_node_->get_clock());
            tf_listener_ =
                std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        }

        bool NavPlanner::on_configure()
        {
            m_global_map_subscriber = this->m_node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
                "/roar/map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
                std::bind(&NavPlanner::onReceiveGlobalMap, this, std::placeholders::_1));
            m_goal_pose_subscriber = this->m_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
                "/goal_pose", rclcpp::QoS(1).reliable(),
                std::bind(&NavPlanner::onReceiveGoalPose, this, std::placeholders::_1));

            // diagnostic message publisher 
            m_diagnostic_publisher = this->m_node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(1).reliable());
            RCLCPP_INFO(m_logger_, "NavPlanner is configured");
            return true;
        }

        bool NavPlanner::on_activate()
        {
            RCLCPP_INFO(m_logger_, "NavPlanner is activated");
            m_diagnostic_publisher->on_activate();
            return true;
        }

        void NavPlanner::onReceiveGoalPose(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
        {
            RCLCPP_INFO(m_logger_, "Received goal pose");
            m_goal_pose_stamped = msg;
            // print goal pose, with header
            RCLCPP_INFO(m_logger_, "Goal pose: frame: %s | x: %f, y: %f, z: %f", msg->header.frame_id.c_str(), msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

            // transform goal pose to map frame
            m_goal_pose_stamped = p_transformGoalPoseToMap(m_goal_pose_stamped, m_global_map);
            if (m_goal_pose_stamped == nullptr)
            {
                RCLCPP_ERROR(m_logger_, "Failed to transform goal pose to map frame");
                // send diagnostic message
                this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "[NavPlanner] GoalPoseTransform", "Failed to transform goal pose to map frame");
                this->p_sendDiagMessage();
                return;
            }
            didGoalPoseUpdated = true;
        }

        StepResult NavPlanner::step(const StepInput input)
        {
            StepResult stepResult;
            // construct diagnostic message
            this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "[NavPlanner] is GoalPoseUpdated", didGoalPoseUpdated ? "true" : "false");
            this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "[NavPlanner] ReceiveGoalPose", didReceiveGoalPose() ? "true" : "false");
            this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "[NavPlanner] GlobalMap", checkGlobalMap() ? "true" : "false");
            this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "[NavPlanner] GoalWithinGlobalMap", checkGoalWithinGlobalMap() ? "true" : "false");
            this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "[NavPlanner] VehicleStatus", checkVehicleStatus(input) ? "true" : "false");
            this->p_sendDiagMessage();

            if (didGoalPoseUpdated && didReceiveGoalPose() && checkGlobalMap() && checkGoalWithinGlobalMap() && checkVehicleStatus(input))
            {
                RCLCPP_DEBUG_STREAM(m_logger_, "All inputs received");
                // if have never computed global path, compute it
                NavPlannerGlobalPathFinderInputs planning_inputs;
                planning_inputs.odom = input.odom;
                planning_inputs.global_map = m_global_map;
                planning_inputs.goal_pose = m_goal_pose_stamped;
                NavPlannerGlobalPathFinderOutput planning_outputs = planTrajectory(planning_inputs);
                if (planning_outputs.status && planning_outputs.global_path != nullptr)
                {
                    RCLCPP_DEBUG_STREAM(m_logger_, "Path planned successfully");
                    m_global_path = planning_outputs.global_path;

                    stepResult.global_path = planning_outputs.global_path;
                    stepResult.status = true;
                }
                else
                {
                    RCLCPP_ERROR(m_logger_, "Failed to plan trajectory");
                    stepResult.status = false;
                    didGoalPoseUpdated = false;
                    return stepResult;
                }
            }
            didGoalPoseUpdated = false; // indicate that this goal pose is processed

            if (m_global_path == nullptr)
            {
                // emit error
                // RCLCPP_ERROR(m_logger_, "Global path is not computed");
                p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "[NavPlanner] GlobalPath", "Global path is not computed");
                p_sendDiagMessage();
                stepResult.status = false;
                return stepResult;
            }
            // p_debugPath(m_global_path);
            stepResult.status = true;
            stepResult.global_path = m_global_path;
            return stepResult;
        }
        bool NavPlanner::on_deactivate()
        {
            m_diagnostic_publisher->on_deactivate();
            return true;
        }

        bool NavPlanner::checkGoalWithinGlobalMap()
        {
            if (m_global_map == nullptr || m_goal_pose_stamped == nullptr)
            {
                return false;
            }
            // check if goal pose is within global map
            if (m_goal_pose_stamped->pose.position.x < m_global_map->info.origin.position.x ||
                m_goal_pose_stamped->pose.position.y < m_global_map->info.origin.position.y ||
                m_goal_pose_stamped->pose.position.x > m_global_map->info.origin.position.x + m_global_map->info.width * m_global_map->info.resolution ||
                m_goal_pose_stamped->pose.position.y > m_global_map->info.origin.position.y + m_global_map->info.height * m_global_map->info.resolution)
            {
                RCLCPP_ERROR(m_logger_, "Goal pose is not within global map");
                return false;
            }
            return true;
        }

        geometry_msgs::msg::PoseStamped::SharedPtr NavPlanner::p_transformGoalPoseToMap(const geometry_msgs::msg::PoseStamped::SharedPtr goal_pose, const nav_msgs::msg::OccupancyGrid::SharedPtr global_map)
        {
            std::string targetFrame = global_map->header.frame_id;
            std::string sourceFrame = goal_pose->header.frame_id;

            if (targetFrame == sourceFrame)
            {
                return goal_pose;
            }
            RCLCPP_INFO(m_logger_, "Transforming goal pose from [%s] to [%s]", sourceFrame.c_str(), targetFrame.c_str());
            geometry_msgs::msg::PoseStamped transformed_goal_pose;
            try
            {
                geometry_msgs::msg::TransformStamped transformStamped = tf_buffer_->lookupTransform(targetFrame, sourceFrame, rclcpp::Time(0));
                tf2::doTransform(*goal_pose, transformed_goal_pose, transformStamped);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_ERROR(m_logger_, "Failed to transform goal pose from %s to %s: %s", sourceFrame.c_str(), targetFrame.c_str(), ex.what());
                return nullptr;
            }
            RCLCPP_DEBUG(m_logger_, "Goal pose transformed successfully");
            return std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_goal_pose);

        }

        NavPlanner::NavPlannerGlobalPathFinderOutput NavPlanner::planTrajectory(const NavPlannerGlobalPathFinderInputs &inputs)
        {
            NavPlannerGlobalPathFinderOutput outputs;
            // check if inputs are all filled
            if (inputs.global_map == nullptr || inputs.odom == nullptr || inputs.goal_pose == nullptr)
            {
                outputs.status = false;
                RCLCPP_ERROR(m_logger_, "Inputs for global path finder is not valid");
                return outputs;
            }

            RCLCPP_DEBUG_STREAM(m_logger_, "Start planning global trajectory");
            p_debugGlobalTrajectoryInputs(inputs);

            if (m_nav_planner_algo == nullptr)
            {
                RCLCPP_ERROR(m_logger_, "Nav planner algo is not initialized");
                outputs.status = false;
                return outputs;
            }
            // configure
            std::map<std::string, boost::any> configuration;
            bool status = m_nav_planner_algo->configure(configuration);
            if (!status)
            {
                RCLCPP_ERROR(m_logger_, "Failed to configure nav planner algo");
                this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "[NavPlanner] Algo Run Result", "Failed to configure nav planner algo");
                this->p_sendDiagMessage();
                outputs.status = false;
                return outputs;
            }
            RCLCPP_DEBUG_STREAM(m_logger_, "Configured nav planner algo");

            // set inputs for nav planner algo
            NavPlannerInputs::SharedPtr nav_planner_inputs = std::make_shared<NavPlannerInputs>();
            nav_planner_inputs->start = std::make_shared<geometry_msgs::msg::Pose>(inputs.odom->pose.pose);
            nav_planner_inputs->goal = std::make_shared<geometry_msgs::msg::Pose>(inputs.goal_pose->pose);
            nav_planner_inputs->map = inputs.global_map;
            m_nav_planner_algo->setInputs(nav_planner_inputs);

            RCLCPP_DEBUG(m_logger_, "Inputs for nav planner algo is set");

            // plan
            nav_msgs::msg::Path::SharedPtr global_path = m_nav_planner_algo->plan();

            if (global_path == nullptr)
            {
                RCLCPP_ERROR(m_logger_, "Failed to plan global path");
                this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "[NavPlanner] Algo Run Result", "Failed to plan global path");
                this->p_sendDiagMessage();
                outputs.status = false;
                return outputs;
            }
            // set every pose to map frame
            for (auto &pose : global_path->poses)
            {
                pose.header.frame_id = inputs.global_map->header.frame_id;
            }
            global_path->header.frame_id = inputs.global_map->header.frame_id;
            global_path->header.stamp = this->m_node_->get_clock()->now();
            outputs.status = true;
            outputs.global_path = global_path;
            RCLCPP_DEBUG(m_logger_, "Global path planned successfully: %d points", global_path->poses.size());

            this->p_appendDiagStatus(diagnostic_msgs::msg::DiagnosticStatus::OK, "[NavPlanner] Algo Run Result", "Global path planned successfully");
            this->p_sendDiagMessage();

            // reset
            // m_nav_planner_algo->reset();
            return outputs;
        }
    }
}