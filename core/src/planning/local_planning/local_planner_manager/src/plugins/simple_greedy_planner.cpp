#ifndef ROAR_PLANNING__PLUGIN__GREEDY_PLANNER_PLUGIN_CPP_
#define ROAR_PLANNING__PLUGIN__GREEDY_PLANNER_PLUGIN_CPP_

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "local_planner_manager/local_planner_manager_state.hpp"
#include "local_planner_manager/local_planner_plugin_interface.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include <algorithm> // For find
#include <cmath>     // For hypot (Euclidean distance)
#include <limits>
#include <queue>
#include <unordered_map>
#include <cmath>
#include <stack>
#include <set>

using namespace std::chrono_literals;

namespace roar
{
  namespace planning
  {
    namespace local
    {

      class SimpleGreedyPlanner : public LocalPlannerPlugin
      {

      public:
        struct State
        {
          nav_msgs::msg::Odometry::SharedPtr latest_odom;
          nav_msgs::msg::Path::SharedPtr global_plan_;
          nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map;
        };

        template <typename T>
        struct Point
        {
          T x, y;

          Point(T x, T y) : x(x), y(y) {}

          bool operator==(const Point &p) const
          {
            return x == p.x && y == p.y;
          }

          double distance(const Point &p) const
          {
            return std::hypot(x - p.x, y - p.y);
          }

          std::string to_string() const
          {
            return "(" + std::to_string(x) + ", " + std::to_string(y) + ")";
          }
          bool operator<(const Point &p) const
          {
            if (x == p.x)
              return y < p.y;
            return x < p.x;
          }
          bool isInMap(int width, int height) const
          {
            return x >= 0 && x < width && y >= 0 && y < height;
          }
        };

        struct Config
        {
          double next_waypoint_dist;
          bool should_loop_on_finish = true;
          double goal_tolerance = 0.1;
          bool debug = false;
          int max_iter = 10;
        };
        struct pair_hash
        {
          template <class T1, class T2>
          std::size_t operator()(const std::pair<T1, T2> &pair) const
          {
            auto hash1 = std::hash<T1>{}(pair.first);
            auto hash2 = std::hash<T2>{}(pair.second);
            return hash1 ^ hash2;
          }
        };

        /**
         * @brief A structure to represent a state or node in the search space
         */
        struct Node
        {
          int x, y;                     // Grid coordinates
          double g, h;                  // g: cost from start, h: heuristic cost to goal
          std::shared_ptr<Node> parent; // Using smart pointers for parent nodes

          Node(int x, int y, double g, double h, std::shared_ptr<Node> parent = nullptr)
              : x(x), y(y), g(g), h(h), parent(parent) {}

          double f() const { return h; }
          friend bool operator<(const Node &a, const Node &b) { return a.f() < b.f(); }
          double distance(const Point<double> &p) const
          {
            return std::hypot(x - p.x, y - p.y);
          }
          double distance(const Point<int> &p) const
          {
            return std::hypot(x - p.x, y - p.y);
          }
          double distance(const Node &n) const
          {
            return std::hypot(x - n.x, y - n.y);
          }
          
        };

        struct CompareNode
        {
          bool operator()(const std::shared_ptr<Node> &a, const std::shared_ptr<Node> &b) const
          {
            return a->f() > b->f(); // For min-heap based on heuristic
          }
        };

        SimpleGreedyPlanner() = default;
        ~SimpleGreedyPlanner() = default;

        void initialize(nav2_util::LifecycleNode *node) override
        {
          LocalPlannerPlugin::initialize(node);

          RCLCPP_INFO(logger_, "Initializing SimpleGreedyPlanner");
          node_ = node;
          this->m_state_ = std::make_shared<State>();
          this->config_ = Config{
              node_->declare_parameter<double>(
                  "SimpleGreedyPlanner.next_waypoint_dist", 5.0),
              node_->declare_parameter<bool>(
                  "SimpleGreedyPlanner.should_loop_on_finish", true),
              node_->declare_parameter<double>("SimpleGreedyPlanner.goal_tolerance",
                                               0.1),
              node_->declare_parameter<bool>("SimpleGreedyPlanner.debug", false),
              node_->declare_parameter<int>("SimpleGreedyPlanner.max_iter", 10),
          };
          if (this->config_.debug)
          {
            RCLCPP_INFO_STREAM(logger_, "Setting debug level for "
                                            << this->get_plugin_name() << " to DEBUG");
            bool _ = rcutils_logging_set_logger_level(this->get_plugin_name(),
                                                      RCUTILS_LOG_SEVERITY_DEBUG);
          }

          // print config
          RCLCPP_INFO_STREAM(
              logger_, "SimpleGreedyPlanner config: \n"
                           << "SimpleGreedyPlanner.next_waypoint_dist: " << this->config_.next_waypoint_dist << "\n"
                           << "SimpleGreedyPlanner.debug: " << this->config_.debug << "\n"
                           << "SimpleGreedyPlanner.goal_tolerance: " << this->config_.goal_tolerance << "\n"
                           << "SimpleGreedyPlanner.max_iter: " << this->config_.max_iter << "\n"
                           << "\n");
        }

        const char *get_plugin_name() override { return "SimpleGreedyPlanner"; }

        bool configure(const LocalPlannerManagerConfig::SharedPtr config) override
        {
          RCLCPP_DEBUG(logger_, "Configuring SimpleGreedyPlanner");
          this->config_.should_loop_on_finish = config->should_loop_on_finish;
          return true;
        }

        bool update(const LocalPlannerManagerState::SharedPtr state) override
        {
          RCLCPP_DEBUG(logger_, "Updating SimpleGreedyPlanner");
          this->m_state_->latest_odom = state->odom;
          this->m_state_->global_plan_ = state->global_plan;
          this->m_state_->occupancy_map = state->occupancy_map;
          return true;
        }

        nav_msgs::msg::Path::SharedPtr compute() override
        {
          RCLCPP_DEBUG(logger_, "----------Computing SimpleGreedyPlanner----------");
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
          if (this->getTfBuffer() == nullptr)
          {
            RCLCPP_DEBUG_STREAM(logger_, "tf_buffer is null, not computing");
            return nullptr;
          }

          nav_msgs::msg::Path::SharedPtr path;

          // find the next waypoint
          geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint =
              this->findNextWaypoint(this->config_.next_waypoint_dist,
                                     *this->m_state_->latest_odom,
                                     *this->m_state_->global_plan_);
          if (next_waypoint == nullptr)
          {
            RCLCPP_DEBUG_STREAM(
                logger_, "findNextWaypoint yielded is null, unable to find path");
            return nullptr;
          }
          RCLCPP_DEBUG_STREAM(logger_, "next waypoint: "
                                           << next_waypoint->pose.position.x << ", "
                                           << next_waypoint->pose.position.y << ", "
                                           << next_waypoint->pose.position.z);

          // greedy search
          geometry_msgs::msg::PoseStamped::SharedPtr startPose = std::make_shared<geometry_msgs::msg::PoseStamped>();
          startPose->header = this->m_state_->latest_odom->header;
          startPose->pose = this->m_state_->latest_odom->pose.pose;

          path = this->greedySearch(startPose,
                                    this->m_state_->occupancy_map, 
                                    next_waypoint);

          if (path == nullptr)
          {
            RCLCPP_DEBUG_STREAM(logger_,
                                "greedySearch yielded is null, unable to find path");
            return nullptr;
          }

          return path;
        }

        nav_msgs::msg::Path::SharedPtr
        findPathToNextWaypoint(const float next_waypoint_dist,
                               nav_msgs::msg::Odometry odom,
                               nav_msgs::msg::Path global_plan)
        {
          geometry_msgs::msg::PoseStamped::SharedPtr next_waypoint =
              this->findNextWaypoint(next_waypoint_dist, odom, global_plan);
          if (next_waypoint == nullptr)
          {
            RCLCPP_DEBUG_STREAM(
                logger_, "findNextWaypoint yielded is null, unable to find path");
            return nullptr;
          }

          nav_msgs::msg::Path::SharedPtr path =
              std::make_shared<nav_msgs::msg::Path>();
          path->header = global_plan.header;
          path->header.stamp = node_->now();

          // generate n waypoints from odom to next_waypoint
          int n = 10;
          double dx =
              (next_waypoint->pose.position.x - odom.pose.pose.position.x) / n;
          double dy =
              (next_waypoint->pose.position.y - odom.pose.pose.position.y) / n;
          double dz =
              (next_waypoint->pose.position.z - odom.pose.pose.position.z) / n;

          for (int i = 0; i < n; i++)
          {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = odom.header;
            pose.header.stamp = node_->now();
            pose.pose.position.x = odom.pose.pose.position.x + dx * i;
            pose.pose.position.y = odom.pose.pose.position.y + dy * i;
            pose.pose.position.z = odom.pose.pose.position.z + dz * i;
            path->poses.push_back(pose);
          }
          return path;
        }

        geometry_msgs::msg::PoseStamped::SharedPtr
        findNextWaypoint(const float next_waypoint_min_dist,
                         nav_msgs::msg::Odometry odom,
                         nav_msgs::msg::Path global_plan)
        {
          RCLCPP_DEBUG(logger_, "finding waypoint");

          // find closest waypoint
          // find the closest waypoint to the current position
          double min_distance = std::numeric_limits<double>::max();
          size_t closest_waypoint_index = 0;
          for (size_t i = 0; i < global_plan.poses.size(); i++)
          {
            double distance =
                std::sqrt(std::pow(odom.pose.pose.position.x -
                                       global_plan.poses[i].pose.position.x,
                                   2) +
                          std::pow(odom.pose.pose.position.y -
                                       global_plan.poses[i].pose.position.y,
                                   2));
            if (distance < min_distance)
            {
              min_distance = distance;
              closest_waypoint_index = i;
            }
          }
          RCLCPP_DEBUG_STREAM(logger_, "closest waypoint index: "
                                           << closest_waypoint_index
                                           << ", distance: " << min_distance);

          // find the next waypoint, including looping back to the beginning, if
          // needed double next_waypoint_dist = cte_and_lookahead.second;
          double next_waypoint_dist = next_waypoint_min_dist;
          size_t next_waypoint_index = closest_waypoint_index;
          for (size_t i = 0; i < global_plan.poses.size(); i++)
          {
            size_t next_index =
                (closest_waypoint_index + i) % global_plan.poses.size();

            if (!this->config_.should_loop_on_finish)
            {
              next_index =
                  std::min(closest_waypoint_index + i, global_plan.poses.size() - 1);
            }
            double distance =
                std::sqrt(std::pow(odom.pose.pose.position.x -
                                       global_plan.poses[next_index].pose.position.x,
                                   2) +
                          std::pow(odom.pose.pose.position.y -
                                       global_plan.poses[next_index].pose.position.y,
                                   2));
            if (distance > next_waypoint_dist)
            {
              next_waypoint_index = next_index;
              break;
            }
          }
          RCLCPP_DEBUG_STREAM(logger_,
                              "next waypoint index: " << next_waypoint_index
                                                      << ", next_waypoint_dist: "
                                                      << next_waypoint_dist);
          geometry_msgs::msg::PoseStamped next_waypoint =
              global_plan.poses[next_waypoint_index];

          return std::make_shared<geometry_msgs::msg::PoseStamped>(next_waypoint);
        }

        geometry_msgs::msg::PoseStamped::SharedPtr transformToBaseLink(
            const geometry_msgs::msg::PoseStamped::SharedPtr pose)
        {
          geometry_msgs::msg::PoseStamped transformed_pose;
          try
          {
            geometry_msgs::msg::TransformStamped transform_stamped =
                this->getTfBuffer()->lookupTransform("base_link", pose->header.frame_id, tf2::TimePointZero);
            tf2::doTransform(*pose, transformed_pose, transform_stamped);
          }
          catch (tf2::TransformException &ex)
          {
            RCLCPP_ERROR_STREAM(logger_, "Transform error: " << ex.what());
            return nullptr;
          }
          return std::make_shared<geometry_msgs::msg::PoseStamped>(transformed_pose);
        }

        /**
        * @brief given a node, back trace to find path
        * @param node
        * @return path

        */
        nav_msgs::msg::Path::SharedPtr backTrace(std::shared_ptr<Node> node, Point<int> start_point_occu, nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map)
        {
          auto path = std::make_shared<nav_msgs::msg::Path>();
          path->header.frame_id = "base_link";
          path->header.stamp = node_->now();

          std::shared_ptr<Node> curr_node = node;
          // reconstruct path
          while (curr_node != nullptr)
          {
            geometry_msgs::msg::PoseStamped pose;
            // occu map coord and ros coord is different
            pose.pose.position.x = (curr_node->x - start_point_occu.x)*occupancy_map->info.resolution;
            pose.pose.position.y = (curr_node->y - start_point_occu.y)*occupancy_map->info.resolution;
            pose.pose.position.z = 0;
            path->poses.push_back(pose);
            curr_node = curr_node->parent;
          }

          std::reverse(path->poses.begin(), path->poses.end());

          for (auto &pose : path->poses)
          {
            RCLCPP_DEBUG_STREAM(logger_, "Path: " << pose.pose.position.x << ", " << pose.pose.position.y);
          }

          return path;
        }


        /**
         * @brief Greedy search algorithm to find a path from start to goal, while
         * avoiding obstacles and inflating them by a given distance
         * @param start The starting pose in global frame
         * @param occupancy_map The occupancy map
         * @param goal The goal pose in global frame
         * @param obstacle_inflation_dist The distance to inflate obstacles by (in
         * meters)
         * @return A shared pointer to a Path message, or nullptr if no path is found
         */
        nav_msgs::msg::Path::SharedPtr greedySearch(
            const geometry_msgs::msg::PoseStamped::SharedPtr &start,
            const nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_map,
            const geometry_msgs::msg::PoseStamped::SharedPtr &goal)
        {
          RCLCPP_DEBUG_STREAM(logger_, "start: frame: [map] " << start->pose.position.x << ", " << start->pose.position.y);
          RCLCPP_DEBUG_STREAM(logger_, "goal: frame: [" << goal->header.frame_id << "] " << goal->pose.position.x << ", " << goal->pose.position.y);
          RCLCPP_DEBUG_STREAM(logger_, "occupancy_map: frame: [" << occupancy_map->header.frame_id << "] width: " << occupancy_map->info.width << ", height: " << occupancy_map->info.height << ", res: " << occupancy_map->info.resolution << ", x:" << occupancy_map->info.origin.position.x << ", y: " << occupancy_map->info.origin.position.y);

          // 1. transform start and goal to base_link frame
          std::shared_ptr<geometry_msgs::msg::PoseStamped> start_base_link = transformToBaseLink(start);
          std::shared_ptr<geometry_msgs::msg::PoseStamped> goal_base_link = transformToBaseLink(goal);

          if (start_base_link == nullptr )
          {
            RCLCPP_ERROR_STREAM(logger_, "Failed to transform start to base_link frame");
            return nullptr;
          }
          if (goal_base_link == nullptr )
          {
            RCLCPP_ERROR_STREAM(logger_, "Failed to transform goal to base_link frame");
            return nullptr;
          }
          RCLCPP_DEBUG_STREAM(logger_, "start: frame: [" << start_base_link->header.frame_id << "] " << start_base_link->pose.position.x << ", " << start_base_link->pose.position.y);
          RCLCPP_DEBUG_STREAM(logger_, "goal: frame: [" << goal_base_link->header.frame_id << "] " << goal_base_link->pose.position.x << ", " << goal_base_link->pose.position.y);

          // 1.2 project to base_link frame
          Point<double> start_point = Point(start_base_link->pose.position.x, start_base_link->pose.position.y); // place vehicle at the center of the occupancy map

          RCLCPP_DEBUG_STREAM(logger_, "start_point: " << start_point.to_string().c_str());

          Point<double> end_point = Point(goal_base_link->pose.position.x, goal_base_link->pose.position.y);
          RCLCPP_DEBUG_STREAM(logger_, "end_point: " << end_point.to_string().c_str());

          // 1.3 project to occu map
          Point<int> start_point_occu = Point<int>(int((start_point.x - occupancy_map->info.origin.position.x) / occupancy_map->info.resolution),
                                                   int((start_point.y - occupancy_map->info.origin.position.y) / occupancy_map->info.resolution));
          RCLCPP_DEBUG_STREAM(logger_, "start_point_occu: " << start_point_occu.to_string().c_str());

          Point<int> end_point_occu = Point<int>(int((end_point.x - occupancy_map->info.origin.position.x) / occupancy_map->info.resolution),
                                                 int((end_point.y - occupancy_map->info.origin.position.y) / occupancy_map->info.resolution));
          RCLCPP_DEBUG_STREAM(logger_, "end_point_occu: " << end_point_occu.to_string().c_str());
          
          // 2. compute feasible goal

          if (end_point_occu.x < 0 || end_point_occu.x >= occupancy_map->info.width ||
              end_point_occu.y < 0 || end_point_occu.y >= occupancy_map->info.height)
          {
            end_point_occu.x = std::min(end_point_occu.x, int(occupancy_map->info.width - 1));
            end_point_occu.y = std::min(end_point_occu.y, int(occupancy_map->info.height - 1));
          }
          RCLCPP_DEBUG_STREAM(logger_, "end_point_occu_feasible: " << end_point_occu.to_string().c_str());

          // 3. setup for greedy search, using priority queue
          std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, CompareNode> openSet;
          std::unordered_map<std::pair<int, int>, std::shared_ptr<Node>, pair_hash> visited;

          // 3.1 start node
          auto start_node = std::make_shared<Node>(start_point_occu.x, start_point_occu.y, 0, start_point_occu.distance(end_point_occu));
          openSet.push(start_node);

          // 3.2 setup closest node, in case greey search is not able to reach goal directly
          std::shared_ptr<Node> closest_node_to_goal = nullptr;
          double min_dist = std::numeric_limits<double>::max();

          // 4. A* search
          int iter = 0;
          while (!openSet.empty() && iter < this->config_.max_iter)
          {
            auto current = openSet.top();
            // RCLCPP_DEBUG_STREAM(logger_, "current: " << current->x << ", " << current->y);
            openSet.pop();
            visited[{current->x, current->y}] = current;
            iter += 1;

            // update closest node to goal
            double dist = current->distance(end_point_occu);
            if (dist < min_dist) 
            {
              min_dist = dist;
              closest_node_to_goal = current;
            }


            // check if current is goal
            if (current->distance(end_point_occu) < config_.goal_tolerance)
            {
              // RCLCPP_DEBUG_STREAM(logger_, "Exact Path found");
              return backTrace(current, start_point_occu, occupancy_map);
            }

            // neighbor exploration
            for (int dx = -1; dx <= 1; ++dx)
            {
              for (int dy = -1; dy <= 1; ++dy)
              {
                if (dx == 0 && dy == 0)
                {
                  continue; // no looping on to self
                }

                int nextX = current->x + dx;
                int nextY = current->y + dy;

                Point<int> next_point = Point<int>(nextX, nextY);
                if (next_point.isInMap(occupancy_map->info.width, occupancy_map->info.height) == false )
                {
                  continue;
                }


                // if it is not visited
                if (!visited[{nextX, nextY}])
                {
                  float g = current->g + 1;
                  double reward_to_goal = next_point.distance(end_point_occu);
                  int obstacle_index = nextX + nextY * occupancy_map->info.width;
                  float obstacle_cost = occupancy_map->data[obstacle_index];
                  float forward_cost = reward_to_goal + obstacle_cost;
                  RCLCPP_DEBUG_STREAM(logger_, "  next: " << nextX << ", " << nextY << ", reward_to_goal: " << reward_to_goal << ", obstacle_cost at [" << obstacle_index << "]: " << obstacle_cost << ", forward_cost: " << forward_cost);

                  auto next = std::make_shared<Node>(nextX, nextY, g, forward_cost, current);
                  openSet.push(next);
                  visited[{nextX, nextY}] = next;
                }
                
              }
            }
          }

          // 5. reconstruct path
          // RCLCPP_DEBUG_STREAM(logger_, "No path found, trying closest path");
          if (closest_node_to_goal != nullptr)
          {
            RCLCPP_DEBUG_STREAM(logger_, "Closest node to goal: " << closest_node_to_goal->x << ", " << closest_node_to_goal->y);
            return backTrace(closest_node_to_goal, start_point_occu, occupancy_map);
          }
          else
          {
            RCLCPP_DEBUG_STREAM(logger_, "No path found");
            return nullptr; // Path not found
          }
        }

      private:
        nav2_util::LifecycleNode *node_{};
        std::shared_ptr<State> m_state_;
        Config config_;
      };
    } // namespace local
  }   // namespace planning
} // namespace roar

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(roar::planning::local::SimpleGreedyPlanner,
                       roar::planning::local::LocalPlannerPlugin)

#endif // ROAR_PLANNING__PLUGIN__GREEDY_PLANNER_PLUGIN_CPP_