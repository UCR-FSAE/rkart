#ifndef ROAR_PLANNING__PLUGIN__DUMMY_PLANNER_PLUGIN_CPP_
#define ROAR_PLANNING__PLUGIN__DUMMY_PLANNER_PLUGIN_CPP_

#include "rclcpp/rclcpp.hpp"
#include "local_planner_manager/local_planner_manager_state.hpp"
#include "local_planner_manager/local_planner_plugin_interface.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace roar
{
    namespace planning
    {
        namespace local
        {

            class RacingLinePerturbPlanner : public LocalPlannerPlugin
            {

            public:
                struct State
                {
                    nav_msgs::msg::Odometry::SharedPtr latest_odom;
                    nav_msgs::msg::Path::SharedPtr global_plan_;
                    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map;
                    bool is_in_obstacle_avoidance_state = false;
                    nav_msgs::msg::Path::SharedPtr obstacle_avoidance_path;
                    rclcpp::Time obstacle_avoidance_start_time;

                    geometry_msgs::msg::Point::SharedPtr last_vehicle_pos;
                };

                struct Config
                {
                    double next_waypoint_dist;
                    double obstacle_avoidance_init_distance = 1;
                    double obstacle_detection_distance = 10;
                    long int min_obstacles_detected_along_path = 1;

                    bool should_use_best_effort_find_next_waypoint = true;
                    bool debug = false;
                    double perturbation_dist = 1.0;
                    int obstacle_val = 20;

                    double obstacle_avoidance_timeout = 2.0; // seconds
                };

                RacingLinePerturbPlanner() = default;
                ~RacingLinePerturbPlanner() = default;

                void initialize(nav2_util::LifecycleNode *node) override
                {
                    // call the super class initialize
                    LocalPlannerPlugin::initialize(node);
                    RCLCPP_INFO(node->get_logger(), "Initializing RacingLinePerturbPlanner");
                    node_ = node;
                    this->m_state_ = std::make_shared<State>();
                    this->config_ = Config{
                        node_->declare_parameter<double>("RacingLinePerturbPlanner.next_waypoint_dist", 5.0),
                        node_->declare_parameter<double>("RacingLinePerturbPlanner.obstacle_avoidance_init_distance", 1.0),
                        node_->declare_parameter<double>("RacingLinePerturbPlanner.obstacle_detection_distance", 10.0),
                        node->declare_parameter<long int>("RacingLinePerturbPlanner.min_obstacles_detected_along_path", 1),
                        node_->declare_parameter<bool>("RacingLinePerturbPlanner.should_use_best_effort_find_next_waypoint", true),
                        node_->declare_parameter<bool>("RacingLinePerturbPlanner.debug", false),
                        node_->declare_parameter<double>("RacingLinePerturbPlanner.perturbation_dist", 1.0),
                        node_->declare_parameter<long int>("RacingLinePerturbPlanner.obstacle_val", static_cast<int>(100)),
                        node_->declare_parameter<double>("RacingLinePerturbPlanner.obstacle_avoidance_timeout", 2.0),
                    };

                    if (this->config_.debug)
                    {
                        bool _ = rcutils_logging_set_logger_level(this->get_plugin_name(),
                                                                  RCUTILS_LOG_SEVERITY_DEBUG);
                        RCLCPP_DEBUG_STREAM(logger_, "RacingLinePerturbPlanner config: " << this->config_.next_waypoint_dist);
                    }

                    // print config
                    RCLCPP_INFO_STREAM(logger_, "RacingLinePerturbPlanner config: \n"
                                                     << "RacingLinePerturbPlanner.next_waypoint_dist" << this->config_.next_waypoint_dist << "\n"
                                                     << "RacingLinePerturbPlanner.obstacle_avoidance_init_distance" << this->config_.obstacle_avoidance_init_distance << "\n"
                                                     << "RacingLinePerturbPlanner.obstacle_detection_distance" << this->config_.obstacle_detection_distance << "\n"
                                                     << "RacingLinePerturbPlanner.min_obstacles_detected_along_path" << this->config_.min_obstacles_detected_along_path << "\n"
                                                     << "RacingLinePerturbPlanner.should_use_best_effort_find_next_waypoint" << this->config_.should_use_best_effort_find_next_waypoint << "\n"
                                                     << "RacingLinePerturbPlanner.debug" << this->config_.debug << "\n"
                                                     << "RacingLinePerturbPlanner.perturbation_dist" << this->config_.perturbation_dist << "\n"
                                                     );
                }

                const char *get_plugin_name() override
                {
                    return "RacingLinePerturbPlanner";
                }

                bool configure(const LocalPlannerManagerConfig::SharedPtr config) override
                {
                    RCLCPP_DEBUG(logger_, "Configuring RacingLinePerturbPlanner");
                    return true;
                }

                bool update(const LocalPlannerManagerState::SharedPtr state) override
                {
                    // RCLCPP_DEBUG(logger_, "Updating RacingLinePerturbPlanner");
                    this->m_state_->latest_odom = state->odom;
                    this->m_state_->global_plan_ = state->global_plan;
                    this->m_state_->occupancy_map = state->occupancy_map;
                    return true;
                }

                nav_msgs::msg::Path::SharedPtr compute() override
                {
                    // RCLCPP_DEBUG(logger_, "---------");

                    // RCLCPP_DEBUG(logger_, "Computing RacingLinePerturbPlanner");
                    if (this->m_state_->latest_odom == nullptr)
                    {
                        RCLCPP_DEBUG_STREAM(logger_, "latest_odom is null, not computing");
                        return nullptr;
                    }
                    if (this->m_state_->global_plan_ == nullptr)
                    {
                        RCLCPP_DEBUG_STREAM(logger_, "global_plan_ is null, not computing");
                        return nullptr;
                    }
                    if (this->m_state_->occupancy_map == nullptr)
                    {
                        RCLCPP_DEBUG_STREAM(logger_, "occupancy_map is null, not computing");
                        return nullptr;
                    }

                    // try getting map -> base_link transform
                    geometry_msgs::msg::TransformStamped::SharedPtr map_to_base_link_transform;
                    try
                    {
                        map_to_base_link_transform = std::make_shared<geometry_msgs::msg::TransformStamped>(this->tf_buffer_->lookupTransform(this->m_state_->latest_odom->header.frame_id, this->m_state_->latest_odom->child_frame_id, tf2::TimePointZero));
                        // RCLCPP_DEBUG_STREAM(logger_, this->m_state_->latest_odom->header.frame_id.c_str() << " -> " << this->m_state_->latest_odom->child_frame_id.c_str() << " transform: " << map_to_base_link_transform->transform.translation.x << ", " << map_to_base_link_transform->transform.translation.y << ", " << map_to_base_link_transform->transform.translation.z);
                    }
                    catch (tf2::TransformException &ex)
                    {
                        RCLCPP_ERROR(logger_, "Transform error: %s", ex.what());
                        return nullptr;
                    }

                    // 0. check if we are in the state of obstacle avoidance, print true or false
                    RCLCPP_DEBUG_STREAM(logger_, "0. is_in_obstacle_avoidance_state: " << this->m_state_->is_in_obstacle_avoidance_state ? "true" : "false");

                    // 1. if is in the state of obstacle avoidance, return the calculated path
                    if (this->m_state_->is_in_obstacle_avoidance_state && this->m_state_->obstacle_avoidance_path != nullptr && this->m_state_->obstacle_avoidance_path->poses.size() > 0 && this->m_state_->last_vehicle_pos != nullptr)
                    {
                        // 1.2 check if we should end obstacle avoidance state, by determinig if the vehicle comes to a certain threshold of the last desired waypoint
                        rclcpp::Duration time_elapsed = node_->now() - this->m_state_->obstacle_avoidance_start_time;
                        if (time_elapsed.seconds() > this->config_.obstacle_avoidance_timeout)
                        {
                            RCLCPP_DEBUG_STREAM(logger_, "obstacle avoidance timeout, ending obstacle avoidance state");
                            this->m_state_->is_in_obstacle_avoidance_state = false;
                            this->m_state_->obstacle_avoidance_path = nullptr;
                            return nullptr;
                        } else {
                            // RCLCPP_DEBUG_STREAM(logger_, "is in the state of obstacle avoidance, returning path, time left: " << this->config_.obstacle_avoidance_timeout - time_elapsed.seconds() << " seconds");
                            // move the obstacle avoidance path forward to the current vehicle position
                            nav_msgs::msg::Path::SharedPtr path = std::make_shared<nav_msgs::msg::Path>();
                            path->header = this->m_state_->obstacle_avoidance_path->header;
                            path->header.stamp = node_->now();
                            for (size_t i = 0; i < this->m_state_->obstacle_avoidance_path->poses.size(); i++)
                            {
                                geometry_msgs::msg::PoseStamped pose;
                                pose.header = this->m_state_->obstacle_avoidance_path->header;
                                pose.header.stamp = node_->now();
                                pose.pose.position.x = this->m_state_->obstacle_avoidance_path->poses[i].pose.position.x + this->m_state_->latest_odom->pose.pose.position.x - this->m_state_->last_vehicle_pos->x;
                                pose.pose.position.y = this->m_state_->obstacle_avoidance_path->poses[i].pose.position.y + this->m_state_->latest_odom->pose.pose.position.y - this->m_state_->last_vehicle_pos->y;
                                pose.pose.position.z = 0;// this->m_state_->obstacle_avoidance_path->poses[i].pose.position.z + this->m_state_->latest_odom->pose.pose.position.z - this->m_state_->last_vehicle_pos->z;
                                path->poses.push_back(pose);
                            }
                            this->m_state_->last_vehicle_pos = std::make_shared<geometry_msgs::msg::Point>(this->m_state_->latest_odom->pose.pose.position);
                            this->m_state_->obstacle_avoidance_path = path;
                            return this->m_state_->obstacle_avoidance_path;
                        }

                    } else {
                        this->m_state_->is_in_obstacle_avoidance_state = false;
                        this->m_state_->obstacle_avoidance_path = nullptr;
                    }

                    // 2. if not in obstacle avoidance, find path to the next waypoint
                    // note: path in here is in the global frame
                    nav_msgs::msg::Path::SharedPtr path = this->findPathToNextWaypoint(this->config_.next_waypoint_dist, *this->m_state_->latest_odom, *this->m_state_->global_plan_);
                    if (path == nullptr || path->poses.size() == 0)
                    {
                        RCLCPP_DEBUG_STREAM(logger_, "findPathToNextWaypoint yielded is null, unable to find path");
                        return nullptr;
                    }
                    // RCLCPP_DEBUG_STREAM(logger_, "1. path global: " << path->poses.size() << " poses");
                    // p_debug_path(path);

                    nav_msgs::msg::Path::SharedPtr path_ego_frame = this->transformPathToEgoFrame(path, map_to_base_link_transform);
                    RCLCPP_DEBUG_STREAM(logger_, "1. path_ego_frame: " << path_ego_frame->poses.size() << " poses");
                    p_debug_path(path_ego_frame);
                    
                    // 2.1 find obstacles along the path
                    std::shared_ptr<std::vector<geometry_msgs::msg::Point>> obstacles = std::make_shared<std::vector<geometry_msgs::msg::Point>>();
                    this->findObstaclesInPath(obstacles, path_ego_frame, this->m_state_->occupancy_map, this->config_.obstacle_detection_distance);

                    // 3. see if obstacles are along the path
                    RCLCPP_DEBUG_STREAM(logger_, "2.1. num_obstacles_along_path: " << obstacles->size());

                    if (obstacles != nullptr && obstacles->size() >= this->config_.min_obstacles_detected_along_path)
                    {
                        // 4.1 if obstacle is in the way, generate perturbed paths
                        RCLCPP_DEBUG_STREAM(logger_, "[" << obstacles->size() << "] obstacles detected on path, generating perturbed paths");
                        // generate perturbed paths
                        nav_msgs::msg::Path::SharedPtr best_path = this->generateBestAlternativePath(path_ego_frame, obstacles, this->config_.perturbation_dist);

                        if (best_path == nullptr)
                        {
                            RCLCPP_DEBUG_STREAM(logger_, "p_findBestPath yielded is null, unable to find path");
                            return nullptr;
                        }

                        // transform best path to global frame
                        nav_msgs::msg::Path::SharedPtr best_path_global_frame = this->transformPathToGlobalFrame(best_path, this->m_state_->latest_odom);
                        RCLCPP_DEBUG_STREAM(logger_, "4. best_path_global_frame: " << best_path_global_frame->poses.size() << " poses");
                        this->m_state_->is_in_obstacle_avoidance_state = true;
                        this->m_state_->obstacle_avoidance_path = best_path_global_frame;
                        this->m_state_->obstacle_avoidance_start_time = node_->now();
                        this->m_state_->last_vehicle_pos = std::make_shared<geometry_msgs::msg::Point>(this->m_state_->latest_odom->pose.pose.position);
                        // p_debug_path(best_path_global_frame);
                        return best_path_global_frame;
                    }
                    else
                    {
                        // 4.2 if no obstacle is in the way, return the path to the next waypoint
                        RCLCPP_DEBUG_STREAM(logger_, "no obstacle detected on path, returning path");
                        
                        return path;
                    }
                }

                void findObstaclesInPath(std::shared_ptr<std::vector<geometry_msgs::msg::Point>> obstacles, nav_msgs::msg::Path::SharedPtr path, nav_msgs::msg::OccupancyGrid::SharedPtr occu_map, double obstacle_detection_distance)
                {
                    if (path == nullptr || path->poses.size() == 0)
                    {
                        RCLCPP_DEBUG_STREAM(logger_, "path is null or empty, unable to find obstacles");
                        return;
                    }
                    if (obstacles == nullptr)
                    {
                        obstacles = std::make_shared<std::vector<geometry_msgs::msg::Point>>();
                    }
                    obstacles->clear();

                    // for every point on obstacle map, if it is not 0, it is an obstacle, find its position relative to width/2, height/2, and add to obstacles
                    int x_vehicle = occu_map->info.width / 2;
                    int y_vehicle = occu_map->info.height / 2;

                    // find obstacles in range of vehicle
                    for (size_t j = 0; j < occu_map->data.size(); j++)
                    {
                        for (size_t i = 0; i < path->poses.size(); i++)
                        {
                            if (occu_map->data[j] == this->config_.obstacle_val)
                            {
                                // find the position of the obstacle in the map
                                int x = j % occu_map->info.width;
                                int y = j / occu_map->info.width;
                                
                                // find the distance between the vehicle and the obstacle. obstacle map and path are in the reverse frame
                                double dist_y = (x - x_vehicle) * occu_map->info.resolution;
                                double dist_x = (y - y_vehicle) * occu_map->info.resolution;

                                double dist = std::sqrt(std::pow(path->poses[i].pose.position.x - dist_x, 2) +
                                                        std::pow(path->poses[i].pose.position.y - dist_y, 2));
                                // RCLCPP_DEBUG_STREAM(logger_, "Path i:" << i << " dist: " << dist << ", dist_x: " << dist_x << ", dist_y: " << dist_y << ", x: " << path->poses[i].pose.position.x << ", y: " << path->poses[i].pose.position.y);

                                if (dist < 0.5) {
                                    geometry_msgs::msg::Point obstacle;
                                    obstacle.x = dist_x;
                                    obstacle.y = dist_y;
                                    obstacle.z = 0;
                                    obstacles->push_back(obstacle);
                                }
                            }
                        }
                    }
                }

                nav_msgs::msg::Path::SharedPtr
                transformPathToEgoFrame(nav_msgs::msg::Path::SharedPtr path, geometry_msgs::msg::TransformStamped::SharedPtr transform)
                {
                    nav_msgs::msg::Path::SharedPtr path_ego_frame = std::make_shared<nav_msgs::msg::Path>();
                    path_ego_frame->header = transform->header;
                    path_ego_frame->header.stamp = node_->now();
                    // RCLCPP_DEBUG_STREAM(logger_, "transform: " << transform->transform.translation.x << ", " << transform->transform.translation.y);
                    for (size_t i = 0; i < path->poses.size(); i++)
                    {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = path->header;
                        pose.header.stamp = node_->now();
                        pose.pose.position.x = path->poses[i].pose.position.x - transform->transform.translation.x;
                        pose.pose.position.y = path->poses[i].pose.position.y - transform->transform.translation.y;
                        // pose.pose.position.z = path->poses[i].pose.position.z - transform->transform.translation.z;
                        path_ego_frame->poses.push_back(pose);
                    }
                    return path_ego_frame;
                }

                nav_msgs::msg::Path::SharedPtr transformPathToGlobalFrame(nav_msgs::msg::Path::SharedPtr path, nav_msgs::msg::Odometry::SharedPtr odom)
                {
                    nav_msgs::msg::Path::SharedPtr path_global_frame = std::make_shared<nav_msgs::msg::Path>();
                    path_global_frame->header = odom->header;
                    path_global_frame->header.stamp = odom->header.stamp;

                    for (size_t i = 0; i < path->poses.size(); i++)
                    {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = path->header;
                        pose.header.stamp = odom->header.stamp;
                        pose.pose.position.x = path->poses[i].pose.position.x + odom->pose.pose.position.x;
                        pose.pose.position.y = path->poses[i].pose.position.y + odom->pose.pose.position.y;
                        pose.pose.position.z = path->poses[i].pose.position.z + odom->pose.pose.position.z;
                        path_global_frame->poses.push_back(pose);
                    }
                    return path_global_frame;
                }

                nav_msgs::msg::Path::SharedPtr
                findPathToNextWaypoint(const float next_waypoint_dist, nav_msgs::msg::Odometry odom, nav_msgs::msg::Path global_plan)
                {
                    geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint = this->findNextWaypoint(next_waypoint_dist, odom, global_plan);
                    if (next_waypoint == nullptr)
                    {
                        RCLCPP_DEBUG_STREAM(logger_, "findNextWaypoint yielded is null, unable to find path");
                        return nullptr;
                    }

                    // RCLCPP_DEBUG_STREAM(logger_, "next_waypoint: " << next_waypoint->pose.position.x << ", " << next_waypoint->pose.position.y);

                    nav_msgs::msg::Path::SharedPtr path = std::make_shared<nav_msgs::msg::Path>();
                    path->header = global_plan.header;
                    path->header.stamp = node_->now();

                    // generate n waypoints from odom to next_waypoint
                    int n = int(std::floor(this->config_.next_waypoint_dist));
                    double dx = (next_waypoint->pose.position.x - odom.pose.pose.position.x) / n;
                    double dy = (next_waypoint->pose.position.y - odom.pose.pose.position.y) / n;
                    double dz = (next_waypoint->pose.position.z - odom.pose.pose.position.z) / n;

                    for (int i = 0; i < n; i++)
                    {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = odom.header;
                        pose.header.stamp = node_->now();
                        pose.pose.position.x = odom.pose.pose.position.x + dx * i;
                        pose.pose.position.y = odom.pose.pose.position.y + dy * i;
                        pose.pose.position.z = 0; //odom.pose.pose.position.z + dz * i;
                        path->poses.push_back(pose);
                    }
                    return path;
                }

                geometry_msgs::msg::PoseStamped::SharedPtr findNextWaypoint(const float next_waypoint_min_dist, nav_msgs::msg::Odometry odom, nav_msgs::msg::Path global_plan)
                {
                    size_t next_waypoint_index = this->pFindNextWaypointIndex(next_waypoint_min_dist, odom, global_plan);
                    geometry_msgs::msg::PoseStamped next_waypoint = global_plan.poses[next_waypoint_index];

                    return std::make_shared<geometry_msgs::msg::PoseStamped>(next_waypoint);
                }

                size_t pFindNextWaypointIndex(const float next_waypoint_min_dist, nav_msgs::msg::Odometry odom, nav_msgs::msg::Path global_plan)
                {
                    
                    // RCLCPP_DEBUG_STREAM(logger_, "[pFindNextWaypointIndex] current position: " << odom.pose.pose.position.x << ", " << odom.pose.pose.position.y);
                    // find closest waypoint
                    // find the closest waypoint to the current position
                    double min_distance = std::numeric_limits<double>::max();
                    size_t closest_waypoint_index = 0;
                    for (size_t i = 0; i < global_plan.poses.size(); i++)
                    {
                        size_t next_index = (closest_waypoint_index + i) % global_plan.poses.size();

                        if (!this->config_.should_use_best_effort_find_next_waypoint)
                        {
                            next_index = std::min(closest_waypoint_index + i, global_plan.poses.size() - 1);
                        }
                        
                        double distance = std::sqrt(std::pow(odom.pose.pose.position.x - global_plan.poses[i].pose.position.x, 2) +
                                                    std::pow(odom.pose.pose.position.y - global_plan.poses[i].pose.position.y, 2));
                        if (distance < min_distance)
                        {
                            min_distance = distance;
                            closest_waypoint_index = i;
                        }
                    }
                    // RCLCPP_DEBUG_STREAM(logger_, "[pFindNextWaypointIndex]: closest waypoint: " << global_plan.poses[closest_waypoint_index].pose.position.x << ", " << global_plan.poses[closest_waypoint_index].pose.position.y << " index: " << closest_waypoint_index << ", distance: " << min_distance);

                    // find the next waypoint, including looping back to the beginning, if needed
                    double next_waypoint_dist = next_waypoint_min_dist;
                    size_t next_waypoint_index = closest_waypoint_index;
                    bool did_find_next_waypoint = false;
                    for (size_t i = 0; i < global_plan.poses.size(); i++)
                    {
                        size_t next_index = (closest_waypoint_index + i) % global_plan.poses.size();

                        double distance = std::sqrt(std::pow(global_plan.poses[closest_waypoint_index].pose.position.x - global_plan.poses[next_index].pose.position.x, 2) +
                                                    std::pow(global_plan.poses[closest_waypoint_index].pose.position.y - global_plan.poses[next_index].pose.position.y, 2));

                        // RCLCPP_DEBUG_STREAM(logger_, "i: " << i << "next_index: " << next_index << " closest: " << global_plan.poses[closest_waypoint_index].pose.position.x << ", " << global_plan.poses[closest_waypoint_index].pose.position.y << " next: " << global_plan.poses[next_index].pose.position.x << ", " << global_plan.poses[next_index].pose.position.y << " distance: " << distance);
                        
                        if (distance > next_waypoint_dist)
                        {
                            // RCLCPP_DEBUG(logger_, "[pFindNextWaypointIndex]: found next waypoint");
                            next_waypoint_index = next_index;
                            did_find_next_waypoint = true;
                            break;
                        }
                    }
                    if (!did_find_next_waypoint)
                    {
                        RCLCPP_WARN(logger_, "[pFindNextWaypointIndex]: did not find next waypoint, returning closest waypoint");
                    }

                    // RCLCPP_DEBUG_STREAM(logger_, "[pFindNextWaypointIndex]: next waypoint index: " << next_waypoint_index << ", next_waypoint_dist: " << next_waypoint_dist << ", next waypoint: " << global_plan.poses[next_waypoint_index].pose.position.x << ", " << global_plan.poses[next_waypoint_index].pose.position.y);
                    return next_waypoint_index;
                }

                size_t numObstaclesDetectedAlongPath(nav_msgs::msg::Path::SharedPtr path, nav_msgs::msg::OccupancyGrid::SharedPtr occu_map, double obstacle_avoidance_init_distance)
                {
                    size_t num_obstacles_detected = 0;

                    return num_obstacles_detected;
                }

                /**
                 * @brief generate perturbed paths by adding constant noise to the centralPath
                 */
                nav_msgs::msg::Path::SharedPtr generateBestAlternativePath(nav_msgs::msg::Path::SharedPtr centralPath, std::shared_ptr<std::vector<geometry_msgs::msg::Point>> obstacles, double perturbation_dist = 0.1)
                {
                    std::vector<nav_msgs::msg::Path::SharedPtr> perturbed_paths;
                    perturbed_paths.push_back(this->pGeneratePerturbPath(centralPath, perturbation_dist));
                    perturbed_paths.push_back(this->pGeneratePerturbPath(centralPath, -perturbation_dist));
                    
                    // find the best path
                    nav_msgs::msg::Path::SharedPtr best_path = nullptr;
                    double best_path_cost = std::numeric_limits<double>::max();
                    for (size_t i = 0; i < perturbed_paths.size(); i++)
                    {
                        double path_cost = this->pPathCost(perturbed_paths[i], obstacles);
                        if (path_cost < best_path_cost)
                        {
                            best_path_cost = path_cost;
                            best_path = perturbed_paths[i];
                        }
                    }

                    return best_path;
                }

                nav_msgs::msg::Path::SharedPtr pGeneratePerturbPath(nav_msgs::msg::Path::SharedPtr centralPath, float perturb_dist)
                {
                    nav_msgs::msg::Path::SharedPtr perturbed_path = std::make_shared<nav_msgs::msg::Path>();
                    perturbed_path->header = centralPath->header;
                    perturbed_path->header.stamp = node_->now();
                    for (size_t i = 0; i < centralPath->poses.size(); i++)
                    {
                        geometry_msgs::msg::PoseStamped pose;
                        pose.header = centralPath->header;
                        pose.header.stamp = node_->now();
                        pose.pose.position.x = centralPath->poses[i].pose.position.x + perturb_dist;
                        pose.pose.position.y = centralPath->poses[i].pose.position.y;
                        pose.pose.position.z = centralPath->poses[i].pose.position.z;
                        perturbed_path->poses.push_back(pose);
                    }
                    return perturbed_path;
                }

                /**
                 * @brief compute the cost of a path, this is a O(p*n) operation (p = path length, n = number of obstacles, can be optimized
                 * @param path path to compare, in ego frame
                 * @param obstacles list of obstacles, in ego frame
                 * @return cost of the path
                */
                double pPathCost(nav_msgs::msg::Path::SharedPtr path, std::shared_ptr<std::vector<geometry_msgs::msg::Point>> obstacles)
                {
                    double dist_to_obstacle = 0;
                    for (size_t i = 0; i < path->poses.size(); i++)
                    {
                        for (size_t j = 0; j < obstacles->size(); j++)
                        {
                            double dist = std::sqrt(std::pow(path->poses[i].pose.position.x - obstacles->at(j).x, 2) +
                                                    std::pow(path->poses[i].pose.position.y - obstacles->at(j).y, 2));
                            dist_to_obstacle += dist;
                        }
                    }
                    return dist_to_obstacle;
                }

                void p_debug_obstacles(std::vector<geometry_msgs::msg::Point> obstacles, std::string frame)
                {
                    for (size_t i = 0; i < obstacles.size(); i++)
                    {
                        RCLCPP_DEBUG_STREAM(logger_, "obstacle[" << i << "]: " << obstacles[i].x << ", " << obstacles[i].y << " in " << frame);
                    }
                }
            private:
                nav2_util::LifecycleNode *node_{};
                std::shared_ptr<State> m_state_;
                Config config_;

                void p_debug_path(nav_msgs::msg::Path::SharedPtr path)
                {
                    if (path != nullptr) {
                        for (size_t i = 0; i < path->poses.size(); i++)
                        {
                            RCLCPP_DEBUG_STREAM(logger_, "path[" << i << "]: " << path->poses[i].pose.position.x << ", " << path->poses[i].pose.position.y);
                        }
                    }
                }
            };
        } // namespace local
    }     // namespace planning
} // namespace roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::planning::local::RacingLinePerturbPlanner, roar::planning::local::LocalPlannerPlugin)

#endif // ROAR_PLANNING__PLUGIN__DUMMY_PLANNER_PLUGIN_CPP_