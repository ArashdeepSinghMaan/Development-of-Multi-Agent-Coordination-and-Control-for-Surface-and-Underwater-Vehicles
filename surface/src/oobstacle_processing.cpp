#include "/home/arash/ros2_ws/src/surface/include/surface/oobstacle_processing.hpp"
#include <iostream>
#include <numeric>
#include <cmath>
#include <vector>
#include<algorithm>
std::vector<Obstacle> obstacles;

std::vector<Obstacle> extract_obstacles(const std::vector<double>& lidar_data, double threshold, double angle_min, double angle_increment) {
    std::vector<Obstacle> obstacles;
    std::vector<std::pair<double, double>> current_obstacle;

    for (size_t i = 0; i < lidar_data.size(); ++i) {
        if (!current_obstacle.empty() &&
        std::abs(lidar_data[i] - current_obstacle.back().second) > threshold) {
        if (current_obstacle.size() > 2) {
            // Compute obstacle properties
            Obstacle obs = compute_obstacle_properties(current_obstacle);

            //  Extra check to avoid false positives
            if (obs.distance > 0.1 && obs.distance < 6.0) { // Example valid range
                obstacles.push_back(obs);
            }
        }
        current_obstacle.clear();
    }

        // Convert to (angle, distance) pairs
        double angle = angle_min + i * angle_increment;
        current_obstacle.emplace_back(angle, lidar_data[i]);
    }

    if (!current_obstacle.empty() && current_obstacle.size() > 2) {
        double max_distance = std::max_element(current_obstacle.begin(), current_obstacle.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; })->second;
double min_distance = std::min_element(current_obstacle.begin(), current_obstacle.end(),
        [](const auto& a, const auto& b) { return a.second < b.second; })->second;

if ((max_distance - min_distance) < threshold) { // Obstacle consistency check
Obstacle obs = compute_obstacle_properties(current_obstacle);
if (obs.distance > 0.05 && obs.distance < 30.0) {
obstacles.push_back(obs);
}
}

    }

    return obstacles;
}

Obstacle compute_obstacle_properties(const std::vector<std::pair<double, double>>& points) {
    Obstacle obs;

    // Compute the mean distance
    double sum_distance = 0.0;
    for (const auto& p : points) {
        sum_distance += p.second;
    }
    obs.distance = sum_distance / points.size();

    // Fit a line using least squares (based on actual angles and distances)
    double sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_x2 = 0.0;
    for (const auto& p : points) {
        double x = p.second * cos(p.first); // Polar to Cartesian
        double y = p.second * sin(p.first);
        sum_x += x;
        sum_y += y;
        sum_xy += x * y;
        sum_x2 += x * x;
    }

    double n = points.size();
  
    obs.distance = sum_distance / n;

    // Compute center of obstacle
    obs.center_x = sum_x / n;
    obs.center_y = sum_y / n;
    double slope = (n * sum_xy - sum_x * sum_y) / (n * sum_x2 - sum_x * sum_x);

    // Compute the perpendicular vector (normal)
    double norm_x = -slope;
    double norm_y = 1.0;
    double norm_z=0.0;
    double magnitude = std::sqrt((norm_x * norm_x) +( norm_y * norm_y)+ (norm_z*norm_z));
    
    obs.normal = {norm_x / magnitude, norm_y / magnitude,norm_z/magnitude};

    return obs;
}
