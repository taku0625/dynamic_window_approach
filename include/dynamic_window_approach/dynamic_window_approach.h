#ifndef DYNAMIC_WINDOW_APPROACH_H
#define DYNAMIC_WINDOW_APPROACH_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2_ros/transform_listener.h>
#include "dynamic_window_approach/window.h"

class DynamicWindowApproach
{
public:
    DynamicWindowApproach(ros::NodeHandle& nh);
    ~DynamicWindowApproach();

protected:
    geometry_msgs::Pose2D::Ptr createPose();
    Window createWindow(const geometry_msgs::Twist::Ptr& twist);
    std::vector<geometry_msgs::Point::Ptr> createObstaclePoints(const geometry_msgs::Pose2D::Ptr& pose);
    geometry_msgs::Twist::Ptr selectBestTwist(
        const geometry_msgs::Pose2D::Ptr& pose, const geometry_msgs::Twist::Ptr& twist, const Window& window
    );
    
    std::vector<geometry_msgs::Pose2D::Ptr> predictPath(const geometry_msgs::Pose2D::Ptr& pose, const geometry_msgs::Twist::Ptr& next_twist);
    
    double evaluatePath(const geometry_msgs::Pose2D::Ptr& pose, const std::vector<geometry_msgs::Pose2D::Ptr>& path, const geometry_msgs::Twist::Ptr& next_twist);
    double evaluateTargetHeading(const geometry_msgs::Pose2D::Ptr& next_pose);
    double evaluateClearance(const geometry_msgs::Pose2D::Ptr& pose, const std::vector<geometry_msgs::Pose2D::Ptr>& path);
    double evaluateVelocity(const geometry_msgs::Twist::Ptr& next_twist);

    void goalJudgment(const geometry_msgs::Pose2D::Ptr& pose);

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
    void timerPlannerCallback(const ros::TimerEvent& e);
    void timerListenGoalCallback(const ros::TimerEvent& e);

private:
    ros::NodeHandle& nh_;

    ros::Publisher pub_twist_;

    ros::Subscriber sub_scan_;

    ros::Timer timer_planner_;
    ros::Timer timer_listen_goal_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    geometry_msgs::Twist::Ptr twist_;

    sensor_msgs::LaserScan::ConstPtr laser_scan_;
    geometry_msgs::TransformStamped::Ptr goal_;

    const int PUBLISH_HZ_ = 20;
    const int PATH_POINT_NUM_ = 10;
    const double PREDICT_TIME_ = 3.0;
    const double MAX_LINEAR_ = 0.5;
    const double MIN_LINEAR_ = 0.0;
    const double MAX_ANGULAR_ = 0.3;
    const double MIN_ANGULAR_ = -0.3;
    const double MAX_DELTA_LINEAR_ = 0.2;
    const double MAX_DELTA_ANGULAR_ = 0.2;
    const double LINEAR_DURATION_ = 0.05;
    const double ANGULAR_DURATION_ = 0.05;
    const double TH_GAIN_ = 1.0;
    const double C_GAIN_ = 1.0;
    const double V_GAIN_ = 1.0;
    const double OBSTACLE_POINT_RADIUS_ = 0.2;
    const double GOAL_THRESHOLD_ = 0.1;
    const double X_LIDER_ = 0.5;

    bool scan_flag_ = false;
    bool goal_flag_ = false;
    bool reach_goal_ = false;
    bool flag_ = true;
};

#endif