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
