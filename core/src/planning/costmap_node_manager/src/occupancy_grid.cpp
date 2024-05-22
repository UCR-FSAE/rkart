#include "costmap_node_manager/occupancy_grid.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>

OccupancyGrid::OccupancyGrid(int width, int height, double resolution,
                             std::vector<std::vector<double>> points)
{

    width_ = width;
    height_ = height;
    resolution_ = resolution;

    occupancy_grid_.resize(width_, std::vector<int>(height_, 0));
    this->updateCells(points);
}

OccupancyGrid::~OccupancyGrid() {}


bool OccupancyGrid::updateCells(std::vector<std::vector<int>> &cells)
{
    for (auto cell : cells)
    {
        if (cell.size() != 3)
        {
            return false;
        }
        this->updateCell(cell[0], cell[1], cell[2]);
    }
    return true;
}

bool OccupancyGrid::updateCell(int x, int y, int value)
{
    if (x < 0 || x >= width_ || y < 0 || y >= height_)
    {
        return false;
    }
    occupancy_grid_[x][y] = value;
    return true;
}

/**
 * Assume x, y is counting from ego frame of view and ego is in the center of occupancy grid
 * ********* O *********
 * *********************
 * *********************
 * **********E**********
 * *********************
 * *********************
 * *********************
 */
bool OccupancyGrid::updateCell(double x, double y, int value)
{
    int grid_x = std::floor(((y + width_ / 2) / resolution_));
    int grid_y = std::floor(((x + height_ / 2) / resolution_));

    // bool status = this->updateCell(grid_x, grid_y, value);
    // if (!status)
    // {
    //     RCLCPP_ERROR_STREAM(rclcpp::get_logger("roar.costmap_node_manager"), "OccupancyGrid: Failed to update cell: " << x << ", " << y);
    //     RCLCPP_DEBUG_STREAM(rclcpp::get_logger("roar.costmap_node_manager"), "OccupancyGrid: resolution: " << resolution_ << ", width: " << width_ << ", height: " << height_);
    //     RCLCPP_DEBUG_STREAM(rclcpp::get_logger("roar.costmap_node_manager"), "OccupancyGrid: grid x: " << grid_x << ", grid y: " << grid_y);
    //     RCLCPP_DEBUG_STREAM(rclcpp::get_logger("roar.costmap_node_manager"), "OccupancyGrid: ---------------");
    // }
    // return status;
    return true;
}

/**
 * Assume x, y is counting from ego frame of view and ego is in the center of occupancy grid
 * *****O***O***********
 * *********************
 * *********************
 * **********E**********
 * *********************
 * *********************
 * *********************
 */
bool OccupancyGrid::updateCells(std::vector<std::vector<double>> &cells)
{
    for (auto cell : cells)
    {
        if (cell.size() != 3)
        {
            return false;
        }
        int prob = cell[2] * 100;
        this->updateCell(cell[0], cell[1], prob);
    }
    return true;
}

bool OccupancyGrid::toOccupancyGridMsg(
    nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    msg->info.width = width_;
    msg->info.height = height_;
    msg->info.resolution = resolution_;
    msg->info.origin.position.x = -width_ / 2 * resolution_;
    msg->info.origin.position.y = -height_ / 2 * resolution_;
    msg->info.origin.position.z = 0.0;
    msg->info.origin.orientation.x = 0.0;
    msg->info.origin.orientation.y = 0.0;
    msg->info.origin.orientation.z = 0.0;
    msg->info.origin.orientation.w = 1.0;
    msg->data.resize(width_ * height_);
    for (int i = 0; i < width_; i++)
    {
        for (int j = 0; j < height_; j++)
        {
            msg->data[i * width_ + j] = occupancy_grid_[i][j];
        }
    }
    return true;
}
