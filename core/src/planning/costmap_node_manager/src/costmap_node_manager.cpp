#include "costmap_node_manager/costmap_node_manager.hpp"
#include "nav2_util/node_utils.hpp"

using namespace std::chrono_literals;

namespace costmap_node_manager
{
  CostmapNodeManager::CostmapNodeManager()
      : LifecycleNode("costmap_node_manager", "", true)
  {
    RCLCPP_INFO(get_logger(), "Creating Costmap Manager Node");
    m_config_ =
        std::make_shared<CostmapNodeManagerConfig>(CostmapNodeManagerConfig{
            declare_parameter<int>("width", 100),
            declare_parameter<int>("height", 100),
            declare_parameter<double>("resolution", 0.1),
            declare_parameter<double>("loop_rate", 0.1),
            declare_parameter<double>("inflation_radius", 1.0), // 0  means off
            declare_parameter<double>("inflation_cost_scaling", 2.0),
            declare_parameter<bool>("debug", true),
        });
    ;
    if (m_config_->debug)
    {
      bool _ = rcutils_logging_set_logger_level(
          get_logger().get_name(),
          RCUTILS_LOG_SEVERITY_DEBUG); // enable or disable debug
    }

    RCLCPP_INFO(get_logger(), "Costmap Node Manager Config: ");
    RCLCPP_INFO(get_logger(), "width: %d", m_config_->width);
    RCLCPP_INFO(get_logger(), "height: %d", m_config_->height);
    RCLCPP_INFO(get_logger(), "resolution: %f", m_config_->resolution);
    RCLCPP_INFO(get_logger(), "loop_rate: %f", m_config_->loop_rate);
    RCLCPP_INFO(get_logger(), "inflation_radius: %f", m_config_->inflation_radius);
    RCLCPP_INFO(get_logger(), "inflation_cost_scaling: %f", m_config_->inflation_cost_scaling);
    RCLCPP_INFO(get_logger(), "debug: %d", m_config_->debug);
  }

  CostmapNodeManager::~CostmapNodeManager()
  {
    RCLCPP_INFO(get_logger(), "Destroying ROS2Costmap2DNode");
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_configure(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Configuring");

    // seconds to milliseconds
    int loop_rate_milliseconds = int(this->m_config_->loop_rate * 1000);
    this->execution_timer = this->create_wall_timer(
        std::chrono::milliseconds(loop_rate_milliseconds),
        std::bind(&CostmapNodeManager::execution_callback, this));
    this->odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&CostmapNodeManager::odom_callback, this,
                  std::placeholders::_1));

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    qos.best_effort();
    qos.durability_volatile();
    this->laser_scan_sub_ =
        this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", qos,
            std::bind(&CostmapNodeManager::laser_scan_callback, this,
                      std::placeholders::_1));
    this->costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "/occupancy_map", 10);
    this->footprint_pub_ =
        this->create_publisher<geometry_msgs::msg::PolygonStamped>("/footprint",
                                                                   10);
    // tf buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    RCLCPP_DEBUG_STREAM(get_logger(), "Configured");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_activate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Activating");
    this->costmap_pub_->on_activate();
    this->footprint_pub_->on_activate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_deactivate(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Deactivating");
    if (this->execution_timer)
    {
      this->execution_timer->cancel();
    }
    this->costmap_pub_->on_deactivate();
    this->footprint_pub_->on_deactivate();
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_cleanup(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Cleaning up");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn
  CostmapNodeManager::on_shutdown(const rclcpp_lifecycle::State &state)
  {
    RCLCPP_INFO(get_logger(), "Shutting Down");
    return nav2_util::CallbackReturn::SUCCESS;
  }

  void CostmapNodeManager::execution_callback()
  {
    RCLCPP_DEBUG_STREAM(get_logger(),
                        "------------------------------------------------");
    if (this->latest_laser_scan_ == nullptr ||
        latest_laser_to_base_link_transform == nullptr)
    {
      RCLCPP_DEBUG(get_logger(), "No laser scan received yet");
      return;
    }
    // convert laser scan to 2d points
    std::vector<std::vector<double>> points;
    for (int i = 0; i < this->latest_laser_scan_->ranges.size(); i++)
    {
      if (std::isinf(this->latest_laser_scan_->ranges[i]) ||
          std::isinf(this->latest_laser_scan_->ranges[i]))
      {
        continue;
      }
      double angle = this->latest_laser_scan_->angle_min +
                     i * this->latest_laser_scan_->angle_increment;
      double x = this->latest_laser_scan_->ranges[i] * std::cos(angle);
      double y = this->latest_laser_scan_->ranges[i] * std::sin(angle);
      // RCLCPP_DEBUG_STREAM(get_logger(), "Point: " << x << ", " << y);
      points.push_back({x, y, 1.0});
    }
    RCLCPP_DEBUG_STREAM(get_logger(), "Received " << points.size() << " points");

    // create occupancy grid msg
    auto occupancy_grid = std::make_shared<nav_msgs::msg::OccupancyGrid>();
    occupancy_grid->header.stamp = this->now();
    occupancy_grid->header.frame_id = "base_link";
    occupancy_grid->info.width = this->m_config_->width;
    occupancy_grid->info.height = this->m_config_->height;
    occupancy_grid->info.resolution = this->m_config_->resolution;
    // making vehicle at the center of occupancy grid
    occupancy_grid->info.origin.position.x =
        -this->m_config_->width / 2 * this->m_config_->resolution;
    occupancy_grid->info.origin.position.y =
        -this->m_config_->height / 2 * this->m_config_->resolution;
    occupancy_grid->info.origin.position.z = 0.0;
    occupancy_grid->info.origin.orientation.x = 0.0;
    occupancy_grid->info.origin.orientation.y = 0.0;
    occupancy_grid->info.origin.orientation.z = 0.0;
    occupancy_grid->info.origin.orientation.w = 1.0;
    occupancy_grid->data.resize(this->m_config_->width * this->m_config_->height, 0);

    // update occupancy grid
    updateOccupancyMap(occupancy_grid, points);

    // publish occupancy grid
    this->costmap_pub_->publish(*occupancy_grid);
    this->publish_footprint();
    RCLCPP_DEBUG_STREAM(get_logger(), "occupancy_grid published");
  }

  void CostmapNodeManager::updateOccupancyMap(
      std::shared_ptr<nav_msgs::msg::OccupancyGrid> occupancy_grid,
      std::vector<std::vector<double>> points)
  {
    RCLCPP_DEBUG_STREAM(get_logger(), "Updating occupancy grid with "
                                          << points.size() << " points");
    // update occupancy grid
    for (auto point : points)
    {
      double x = point[0]; // point with respect to sensor
      double y = point[1]; // point with respect to sensor

      double sensor_to_base_link_x =
          latest_laser_to_base_link_transform->transform.translation.x;
      double sensor_to_base_link_y =
          latest_laser_to_base_link_transform->transform.translation.y;

      double obstacle_x = x + sensor_to_base_link_x; // distance from base link to obstacle
      double obstacle_y = y + sensor_to_base_link_y;


      double min_grid_x = occupancy_grid->info.origin.position.x;
      double max_grid_x = occupancy_grid->info.origin.position.x + occupancy_grid->info.width * occupancy_grid->info.resolution;
      double min_grid_y = occupancy_grid->info.origin.position.y;
      double max_grid_y = occupancy_grid->info.origin.position.y + occupancy_grid->info.height * occupancy_grid->info.resolution;

      // check if obstacle falls within the grid
      if (obstacle_x < min_grid_x || obstacle_x > max_grid_x ||
          obstacle_y < min_grid_y || obstacle_y > max_grid_y)
      {
        // obstacle not in grid
        // RCLCPP_DEBUG_STREAM(get_logger(), "Obstacle not in grid: " << obstacle_x << ", " << obstacle_y);
        continue;
      } 

      // find grid index
      int grid_x = obstacle_x / occupancy_grid->info.resolution + occupancy_grid->info.width / 2;
      int grid_y = obstacle_y / occupancy_grid->info.resolution + occupancy_grid->info.height / 2;

      int index = grid_y * occupancy_grid->info.width + grid_x;
      // RCLCPP_DEBUG_STREAM(get_logger(), "Obstacle: " << obstacle_x << ", "<< obstacle_y << " Grid: " << grid_x << ", " << grid_y << " index: " << index);

      if (index >= 0 && index < occupancy_grid->data.size())
      {
        occupancy_grid->data[index] = OBSTACLE;
        // inflate the obstacle if needed
        if (this->m_config_->inflation_radius > 0)
        {
          int num_inflated = 0;
          float inflation_radius = this->m_config_->inflation_radius / occupancy_grid->info.resolution;
          // RCLCPP_DEBUG_STREAM(get_logger(), "Inflating obstacle at (" << grid_x << ", " << grid_y << ") with inflation radius: " << inflation_radius);
          for (float i = -inflation_radius; i <= inflation_radius; i++)
          {
            for (float j = -inflation_radius; j <= inflation_radius; j++)
            {
              if (i == 0 && j == 0)
              {
                continue;
              }
              int new_grid_x = grid_x + i;
              int new_grid_y = grid_y + j;
              int new_index = new_grid_y * occupancy_grid->info.width + new_grid_x;
              if (new_grid_x < 0 || new_grid_x >= occupancy_grid->info.width ||
                  new_grid_y < 0 || new_grid_y >= occupancy_grid->info.height)
              {
                continue;
              }
              // RCLCPP_DEBUG_STREAM(get_logger(), "Inflating obstacle at (" << new_grid_x << ", " << new_grid_y << ") with index: " << new_index);
              if (new_index >= 0 && new_index < occupancy_grid->data.size() && new_index != index)
              {
                int decay = OBSTACLE * (this->m_config_->inflation_cost_scaling * std::sqrt(i * i + j * j));
                int cost = OBSTACLE - decay;
                if (cost > occupancy_grid->data[new_index])
                {
                  occupancy_grid->data[new_index] = cost;
                  // RCLCPP_DEBUG_STREAM(get_logger(), "Inflating obstacle at (" << new_grid_x << ", " << new_grid_y << ") with index: " << new_index << " cost: " << cost << " decay:" << decay << " occupancy_grid->data[new_index]:" << int(occupancy_grid->data[new_index]));
                }
                num_inflated++;
              }
            }
          }
          }
        }

      
    }
  }

  void CostmapNodeManager::publish_footprint()
  {
    if (latest_footprint_ == nullptr)
    {
      latest_footprint_ = std::make_shared<geometry_msgs::msg::PolygonStamped>();
      latest_footprint_->header.stamp = this->now();
      latest_footprint_->header.frame_id = "base_link";

      // create a polygon with
      // "[[-0.5,-0.5],[-0.5,0.5],[0.5,0.5],[1.5,0.0],[0.5,-0.5]]"
      geometry_msgs::msg::Polygon polygon = geometry_msgs::msg::Polygon();
      geometry_msgs::msg::Point32 point = geometry_msgs::msg::Point32();
      point.x = -0.5;
      point.y = -0.5;
      polygon.points.push_back(point);
      point.x = -0.5;
      point.y = 0.5;
      polygon.points.push_back(point);
      point.x = 0.5;
      point.y = 0.5;
      polygon.points.push_back(point);
      point.x = 1.5;
      point.y = 0.0;
      polygon.points.push_back(point);
      point.x = 0.5;
      point.y = -0.5;
      polygon.points.push_back(point);
      latest_footprint_->polygon = polygon;
    }

    this->footprint_pub_->publish(*latest_footprint_);
    // RCLCPP_DEBUG_STREAM(get_logger(), "Footprint published");
  }
} // namespace costmap_node_manager

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<costmap_node_manager::CostmapNodeManager>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}