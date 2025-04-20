#ifndef OOBSTACLE_PROCESSING_HPP
#define OOBSTACLE_PROCESSING_HPP

#include <vector>
#include <cmath>

 struct  Obstacle {
     double distance;
    std::vector<double> normal;
     double center_x;
    double center_y;
};

extern std::vector<Obstacle> obstacles;

std::vector<Obstacle> extract_obstacles(const std::vector<double>& lidar_data, double threshold, double angle_min, double angle_increment);
Obstacle compute_obstacle_properties(const std::vector<std::pair<double, double>>& points);


#endif // OBSTACLE_PROCESSING_HPP
