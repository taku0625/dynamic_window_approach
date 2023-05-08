#include "dynamic_window_approach/dynamic_window_approach.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "dynamic_window_approach_node");
    ros::NodeHandle nh;
    DynamicWindowApproach dwa(nh);
    ros::spin();

    return 0;
}