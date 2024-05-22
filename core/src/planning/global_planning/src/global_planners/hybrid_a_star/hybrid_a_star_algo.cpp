
#include "global_planning/planners/hybrid_a_star/hybrid_a_star_algo.hpp"

namespace ROAR
{
    namespace GlobalPlanning
    {
        HybridAStarAlgo::HybridAStarAlgo()
        {
        }

        HybridAStarAlgo::~HybridAStarAlgo()
        {
        }

        bool HybridAStarAlgo::init(rclcpp_lifecycle::LifecycleNode *parent)
        {
            if (parent == nullptr)
            {
                RCLCPP_ERROR(this->m_node_->get_logger(), "Parent is nullptr");
                return false;
            }
            this->m_node_ = parent;

            m_parameters = std::make_shared<HybridAStarAlgoParameters>(
                HybridAStarAlgoParameters{
                    .steering_angle = static_cast<double>(m_node_->declare_parameter<double>("nav_planner.hybrid_a_star_algo.steering_angle", 10.0)),
                    .steering_angle_discrete_num = static_cast<int>(m_node_->declare_parameter<int>("nav_planner.hybrid_a_star_algo.steering_angle_discrete_num", 1)),
                    .segment_length = static_cast<double>(m_node_->declare_parameter<double>("nav_planner.hybrid_a_star_algo.segment_length", 1.6)),
                    .segment_length_discrete_num = static_cast<int>(m_node_->declare_parameter<int>("nav_planner.hybrid_a_star_algo.segment_length_discrete_num", 8)),
                    .wheel_base = static_cast<double>(m_node_->declare_parameter<double>("nav_planner.hybrid_a_star_algo.wheel_base", 1.0)),
                    .steering_penalty = static_cast<double>(m_node_->declare_parameter<double>("nav_planner.hybrid_a_star_algo.steering_penalty", 1.06)),
                    .reversing_penalty = static_cast<double>(m_node_->declare_parameter<double>("nav_planner.hybrid_a_star_algo.reversing_penalty", 2.0)),
                    .steering_change_penalty = static_cast<double>(m_node_->declare_parameter<double>("nav_planner.hybrid_a_star_algo.steering_change_penalty", 1.5)),
                    .shot_distance = static_cast<double>(m_node_->declare_parameter<double>("nav_planner.hybrid_a_star_algo.shot_distance", 5.0)),
                    .grid_size_phi = static_cast<int>(m_node_->declare_parameter<int>("nav_planner.hybrid_a_star_algo.grid_size_phi", 72))});
            return true;
        }

        bool HybridAStarAlgo::configure(const std::map<std::string, boost::any> configuration)
        {
            if (m_parameters == nullptr)
            {
                RCLCPP_ERROR(this->m_node_->get_logger(), "Parameters are not set");
                return false;
            }
            kinodynamic_astar_searcher_ptr_ = std::make_shared<HybridAStar>(
                m_parameters->steering_angle,
                m_parameters->steering_angle_discrete_num,
                m_parameters->segment_length,
                m_parameters->segment_length_discrete_num,
                m_parameters->wheel_base,
                m_parameters->steering_penalty,
                m_parameters->reversing_penalty,
                m_parameters->steering_change_penalty,
                m_parameters->shot_distance,
                m_parameters->grid_size_phi);
            return true;
        }

        bool HybridAStarAlgo::setStart(std::shared_ptr<geometry_msgs::msg::Pose> start)
        {
            this->m_inputs->start = start;
            return true;
        }

        bool HybridAStarAlgo::setGoal(std::shared_ptr<geometry_msgs::msg::Pose> goal)
        {
            this->m_inputs->goal = goal;
            return true;
        }

        bool HybridAStarAlgo::setMap(nav_msgs::msg::OccupancyGrid::SharedPtr map)
        {
            this->m_inputs->map = map;
            return true;
        }

        bool HybridAStarAlgo::setInputs(std::shared_ptr<NavPlannerInputs> inputs)
        {
            this->m_inputs = inputs;
            return true;
        }

        nav_msgs::msg::Path::SharedPtr HybridAStarAlgo::plan()
        {
            // sanity checks
            if (this->m_inputs->start == nullptr || this->m_inputs->goal == nullptr || this->m_inputs->map == nullptr)
            {
                RCLCPP_ERROR(this->m_node_->get_logger(), "Inputs are not set");
                return nullptr;
            }

            // init
            const double map_resolution = 0.2;
            kinodynamic_astar_searcher_ptr_->Init(
                this->m_inputs->map->info.origin.position.x,
                1.0 * this->m_inputs->map->info.resolution * this->m_inputs->map->info.width,
                this->m_inputs->map->info.origin.position.y,
                1.0 * this->m_inputs->map->info.resolution * this->m_inputs->map->info.height,
                this->m_inputs->map->info.resolution,
                map_resolution);


            // get start and goal yaw
            double start_yaw = getYawFromQuaternion(this->m_inputs->start->orientation);
            double goal_yaw = getYawFromQuaternion(this->m_inputs->goal->orientation);

            // convert start and goal to Vec3d
            Vec3d start_raw = Vec3d(this->m_inputs->start->position.x, this->m_inputs->start->position.y, start_yaw);
            Vec3d goal_raw = Vec3d(this->m_inputs->goal->position.x, this->m_inputs->goal->position.y, goal_yaw);
            RCLCPP_DEBUG(this->m_node_->get_logger(), "start_raw: %f, %f, %f", start_raw.x(), start_raw.y(), start_raw.z());
            RCLCPP_DEBUG(this->m_node_->get_logger(), "goal_raw: %f, %f, %f", goal_raw.x(), goal_raw.y(), goal_raw.z());

            // cap start and goal to map boundaries
            Vec3d start = kinodynamic_astar_searcher_ptr_->CapCoordinatesToMapSizeIfNeeded(start_raw);
            Vec3d goal = kinodynamic_astar_searcher_ptr_->CapCoordinatesToMapSizeIfNeeded(goal_raw);
            RCLCPP_DEBUG(this->m_node_->get_logger(), "start: %f, %f, %f", start.x(), start.y(), start.z());
            RCLCPP_DEBUG(this->m_node_->get_logger(), "goal: %f, %f, %f", goal.x(), goal.y(), goal.z());

            RCLCPP_INFO(this->m_node_->get_logger(), "Map initialized, setting obstacles...");

            // set setting obstacle
            unsigned int map_w = kinodynamic_astar_searcher_ptr_->getMapGridSizeX();
            unsigned int map_h = kinodynamic_astar_searcher_ptr_->getMapGridSizeY();
            uint32_t num_obstacles = 0;
            for (unsigned int w = 0; w < map_w; ++w)
            {
                for (unsigned int h = 0; h < map_h; ++h)
                {
                    auto x = static_cast<unsigned int>((w + 0.5) * map_resolution / this->m_inputs->map->info.resolution);
                    auto y = static_cast<unsigned int>((h + 0.5) * map_resolution / this->m_inputs->map->info.resolution);

                    if (this->m_inputs->map->data[y * this->m_inputs->map->info.width + x])
                    {

                        bool status = kinodynamic_astar_searcher_ptr_->SetObstacle(w, h);
                        if (status) {
                            num_obstacles++;
                        }
                    }
                }
            }

            RCLCPP_INFO(this->m_node_->get_logger(), "[%d] Obstacles set", num_obstacles);
            // search
            nav_msgs::msg::Path::SharedPtr path_msg = std::make_shared<nav_msgs::msg::Path>();
            if (kinodynamic_astar_searcher_ptr_->Search(start, goal, 5000000))
            {
                // get path
                VectorVec3d path = kinodynamic_astar_searcher_ptr_->GetPath();
                path_msg->header.frame_id = "map";
                for (auto &point : path)
                {
                    geometry_msgs::msg::PoseStamped pose;
                    pose.pose.position.x = point.x();
                    pose.pose.position.y = point.y();
                    pose.pose.position.z = 0.0;
                    path_msg->poses.push_back(pose);
                }
                return path_msg;
            }
            else {
                RCLCPP_ERROR(this->m_node_->get_logger(), "Failed to find path");
            }

            if (path_msg->poses.size() > 0)
            {
                return path_msg;
            } else {
                RCLCPP_ERROR(this->m_node_->get_logger(), "Path is empty");
                return nullptr;
            }
        }

        void HybridAStarAlgo::reset()
        {
            this->m_inputs = nullptr;
        }
    } // namespace GlobalPlanning
} // namespace ROAR