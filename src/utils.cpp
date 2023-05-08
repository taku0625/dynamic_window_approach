#include "dynamic_window_approach/utils.h"
#include <algorithm>
#include <cmath>
#include <limits>

#include <ros/ros.h>

using namespace std;

double normalizeAngle(double angle)
{
    angle = std::fmod(angle, 2 * M_PI);
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

std::vector<std::vector<double>> minMaxNormalize(const std::vector<std::vector<double>>& v)
{
    std::vector<std::vector<double>> normalized_v(v.size(), std::vector<double>(v[0].size()));

    std::vector<double> row_mines, row_maxes;
    std::transform(v.begin(), v.end(), std::back_inserter(row_mines),
                   [](const auto& row){ return *std::min_element(row.begin(), row.end()); });
    std::transform(v.begin(), v.end(), std::back_inserter(row_maxes),
                   [](const auto& row){ return *std::max_element(row.begin(), row.end()); });
    double min_val = *std::min_element(row_mines.begin(), row_mines.end());
    double max_val = *std::max_element(row_maxes.begin(), row_maxes.end());

    std::transform(v.begin(), v.end(), normalized_v.begin(), [&](const std::vector<double>& row)
    {
        std::vector<double> normalized_row(row.size());
        std::transform(row.begin(), row.end(), normalized_row.begin(), [&](double val)
        {
            return (val - min_val) / (max_val - min_val);
        });
        return normalized_row;
    });

    return normalized_v;
}