#ifndef OCCUPANCY_GRID_HPP
#define OCCANCY_GRID_HPP

#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include <cmath>

template <typename T>
struct Point2d
{
    T x;
    T y;
};

class OccupancyGrid
{
public:
    // constructor from 2d points
    OccupancyGrid(int width, int height, double resolution, std::vector<std::vector<double>> points);

    ~OccupancyGrid();

    bool updateCell(int x, int y, int value);                 // in occupancy grid position
    // vector of cells, each cell is a vector of 3 elements: x, y, value
    bool updateCells(std::vector<std::vector<int>>& cells);    // in occupancy grid position
    bool updateCell(double x, double y, int value);           // in world position
    bool updateCells(std::vector<std::vector<double>>& cells); // in world position
    
    

    int getGridWidth() {
        return width_;
    }
    int getGridHeight()
    {
        return height_;
    }
    double getResolution()
    {
        return resolution_;
    }
    
    std::vector<std::vector<int>> getOccupancyGrid()
    {
        return occupancy_grid_;
    }
    int getOccupancyGridCell(int x, int y)
    {
        return occupancy_grid_[x][y];
    }
    int getOccupancyGridCell(double x, double y)
    {
        int grid_x = std::floor(x * width_ / 2 * resolution_);
        int grid_y = std::floor(y * height_ / 2 * resolution_);
        return occupancy_grid_[grid_x][grid_y];
    }
    int getOccupancyGridCell(int index)
    {
        int x = index / width_;
        int y = index % width_;
        return occupancy_grid_[x][y];
    }

    // to ROS2 occupancy grid message
    bool toOccupancyGridMsg(nav_msgs::msg::OccupancyGrid::SharedPtr msg);

private:
    int width_;
    int height_;
    double resolution_;
    std::vector<std::vector<int>> occupancy_grid_; // 0 = free, 100 = occupied
};

#endif // OCCUPANCY_GRID_HPP