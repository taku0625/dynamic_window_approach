#include "dynamic_window_approach/dynamic_window_approach.h"
#include "dynamic_window_approach/utils.h"
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2/LinearMath/Matrix3x3.h>


using namespace std;


DynamicWindowApproach::DynamicWindowApproach(ros::NodeHandle& nh)
    : nh_(nh), tf_buffer_(), tf_listener_(tf_buffer_)
{
    twist_.reset(new geometry_msgs::Twist());
    goal_.reset(new geometry_msgs::TransformStamped());

    pub_twist_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 1);
    pub_path_points_ = nh_.advertise<geometry_msgs::PoseArray>("/path_points", 1);

    sub_scan_ = nh_.subscribe("/scan", 1, &DynamicWindowApproach::scanCallback, this);

    timer_planner_ = nh_.createTimer(ros::Duration(1 / PUBLISH_HZ_), &DynamicWindowApproach::timerPlannerCallback, this);
    timer_listen_goal_ = nh_.createTimer(ros::Duration(1 / PUBLISH_HZ_), &DynamicWindowApproach::timerListenGoalCallback, this);
    timer_planner_.start();
    timer_listen_goal_.start();
}

DynamicWindowApproach::~DynamicWindowApproach()
{
}

geometry_msgs::Pose2D::Ptr DynamicWindowApproach::createPose()
{
    geometry_msgs::TransformStamped::Ptr base_link_on_map(new geometry_msgs::TransformStamped());
    try
    {
        *base_link_on_map = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
    }
    catch(const tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return nullptr;
    }

    geometry_msgs::Pose2D::Ptr pose(new geometry_msgs::Pose2D);

    pose->x = base_link_on_map->transform.translation.x;
    pose->y = base_link_on_map->transform.translation.y;

    double r, p, y;
    tf2::Quaternion q(0.0, 0.0, 0.0, 1.0);
    q.setZ(base_link_on_map->transform.rotation.z);
    q.setW(base_link_on_map->transform.rotation.w);
    tf2::Matrix3x3(q).getRPY(r, p, y);
    pose->theta = y;

    return pose;
}

Window DynamicWindowApproach::createWindow(const geometry_msgs::Twist::Ptr& twist)
{
    Window window;
    window.max_linear = std::min((twist->linear.x + MAX_DELTA_LINEAR_), MAX_LINEAR_);
    window.min_linear = std::max((twist->linear.x - MAX_DELTA_LINEAR_), MIN_LINEAR_);
    window.max_angular = std::min((twist->angular.z + MAX_DELTA_ANGULAR_), MAX_ANGULAR_);
    window.min_angular = std::max((twist->angular.z - MAX_DELTA_ANGULAR_), MIN_ANGULAR_);
    return window;
}

std::vector<geometry_msgs::Point::Ptr> DynamicWindowApproach::createObstaclePoints(const geometry_msgs::Pose2D::Ptr& pose)
{
    std::vector<geometry_msgs::Point::Ptr> obstacle_points(laser_scan_->ranges.size());
    for(size_t i = 0; i < static_cast<size_t>(laser_scan_->ranges.size()); ++i)
    {
        obstacle_points[i].reset(new geometry_msgs::Point());
        double laser_angle = pose->theta + laser_scan_->angle_min + (double)i * laser_scan_->angle_increment;
        obstacle_points[i]->x = pose->x + laser_scan_->ranges[i] * std::cos(laser_angle);
        obstacle_points[i]->y = pose->y + laser_scan_->ranges[i] * std::sin(laser_angle);
        // obstacle_points[i]->x = pose->x + X_LIDER_ * std::cos(pose->theta) + laser_scan_->ranges[i] * std::cos(laser_angle);
        // obstacle_points[i]->y = pose->y + X_LIDER_ * std::sin(pose->theta) + laser_scan_->ranges[i] * std::sin(laser_angle);
    }
    return obstacle_points;
}

geometry_msgs::Twist::Ptr DynamicWindowApproach::selectBestTwist(
    const geometry_msgs::Pose2D::Ptr& pose, const geometry_msgs::Twist::Ptr& twist, const Window& window
)
{
    geometry_msgs::Twist::Ptr next_twist(new geometry_msgs::Twist());
    int linear_num = (window.max_linear - window.min_linear) / LINEAR_DURATION_ + 1;
    int angular_num = (window.max_angular - window.min_angular) / ANGULAR_DURATION_ + 1;
    std::vector<std::vector<double>> th_values(linear_num, std::vector<double>(angular_num));
    std::vector<std::vector<double>> c_values(linear_num, std::vector<double>(angular_num));
    std::vector<std::vector<double>> v_values(linear_num, std::vector<double>(angular_num));
    std::vector<std::vector<double>> w_values(linear_num, std::vector<double>(angular_num));
    for(size_t i = 0; i < static_cast<size_t>(linear_num); ++i)
    {
        next_twist->linear.x = window.min_linear + (double)i * LINEAR_DURATION_;
        for(size_t j = 0; j < static_cast<size_t>(angular_num); ++j)
        {
            next_twist->angular.z = window.min_angular + (double)j * ANGULAR_DURATION_;
            std::vector<geometry_msgs::Pose2D::Ptr> path = predictPath(pose, next_twist);
            visualizePath(path);
            th_values[i][j] = evaluateTargetHeading(path[path.size() - 1]);
            c_values[i][j] = evaluateClearance(pose, path);
            v_values[i][j] = evaluateVelocity(next_twist);
            w_values[i][j] = evaluateAngularVelocity(next_twist);
        }
    }

    geometry_msgs::Twist::Ptr best_next_twist(new geometry_msgs::Twist());
    double best_evaluation_value = 1000.0;
    for(size_t i = 0; i < static_cast<size_t>(linear_num); ++i)
    {
        next_twist->linear.x = window.min_linear + (double)i * LINEAR_DURATION_;
        for(size_t j = 0; j < static_cast<size_t>(angular_num); ++j)
        {
            next_twist->angular.z = window.min_angular + (double)j * ANGULAR_DURATION_;
            double evaluation_value = TH_GAIN_ * th_values[i][j] + C_GAIN_ * c_values[i][j] + V_GAIN_ * v_values[i][j] + W_GAIN_ * w_values[i][j];
            if(evaluation_value < best_evaluation_value)
            {
                best_evaluation_value = evaluation_value;
                *best_next_twist = *next_twist;
            }
        }
    }
    return best_next_twist;
}

std::vector<geometry_msgs::Pose2D::Ptr> DynamicWindowApproach::predictPath(
    const geometry_msgs::Pose2D::Ptr& pose, const geometry_msgs::Twist::Ptr& next_twist
)
{
    std::vector<geometry_msgs::Pose2D::Ptr> path(PATH_POINT_NUM_ * PREDICT_TIME_);
    double path_point_duration_x = next_twist->linear.x * std::cos(pose->theta) / PUBLISH_HZ_ / PATH_POINT_NUM_;
    double path_point_duration_y = next_twist->linear.y * std::cos(pose->theta) / PUBLISH_HZ_ / PATH_POINT_NUM_;
    double path_point_duration_theta = next_twist->angular.z / PUBLISH_HZ_ / PATH_POINT_NUM_;
    for(size_t i = 0; i < static_cast<size_t>(PATH_POINT_NUM_ * PREDICT_TIME_); ++i)
    {
        path[i].reset(new geometry_msgs::Pose2D());
        path[i]->x = pose->x + path_point_duration_x * (double)(i + 1);
        path[i]->y = pose->y + path_point_duration_y * (double)(i + 1);
        path[i]->theta = pose->theta + path_point_duration_theta * (double)(i + 1);
    }
    return path;
}

double DynamicWindowApproach::evaluateTargetHeading(const geometry_msgs::Pose2D::Ptr& next_pose)
{
    double angle_to_goal = std::atan2(goal_->transform.translation.y - next_pose->y, goal_->transform.translation.x - next_pose->x);
    double delta_theta = angle_to_goal - next_pose->theta;
    double evaluation_value = std::fabs(normalizeAngle(delta_theta)) / M_PI;
    return evaluation_value;
}

double DynamicWindowApproach::evaluateClearance(const geometry_msgs::Pose2D::Ptr& pose, const std::vector<geometry_msgs::Pose2D::Ptr>& path)
{
    double min_distance = std::numeric_limits<double>::infinity();
    std::vector<geometry_msgs::Point::Ptr> obstacle_points = createObstaclePoints(pose);

    for(size_t i = 0; i < static_cast<size_t>(path.size()); ++i)
    {
        for(size_t j = 0; j < static_cast<size_t>(obstacle_points.size()); ++j)
        {
            if(pose->x == obstacle_points[j]->x && pose->y == obstacle_points[j]->y)
            {
                continue;
            }
            double dx = path[i]->x - obstacle_points[j]->x;
            double dy = path[i]->y - obstacle_points[j]->y;
            double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) - OBSTACLE_POINT_RADIUS_;
            if(- 0.05 <= distance && distance <= 0.05)
            {
                return 100.0;
            }
            min_distance = std::min(min_distance, distance);
        }
    }
    return 1.0 / min_distance;
}

double DynamicWindowApproach::evaluateVelocity(const geometry_msgs::Twist::Ptr& next_twist)
{
    return 1.0 - std::fabs(next_twist->linear.x) / MAX_LINEAR_;
}

double DynamicWindowApproach::evaluateAngularVelocity(const geometry_msgs::Twist::Ptr& next_twist)
{
    if(next_twist->linear.x == 0.0)
    {
        return 1.0 - std::fabs(next_twist->angular.z) / MAX_ANGULAR_;
    }
    else
    {
        return 1.0;
    }

}

void DynamicWindowApproach::goalJudgment(const geometry_msgs::Pose2D::Ptr& pose)
{
    double goal_x = goal_->transform.translation.x;
    double goal_y = goal_->transform.translation.y;
    if(
        goal_x - GOAL_THRESHOLD_ < pose->x && pose->x < goal_x + GOAL_THRESHOLD_ &&
        goal_y - GOAL_THRESHOLD_ < pose->y && pose->y < goal_y + GOAL_THRESHOLD_
    )
    {
        reach_goal_ = true;
    }
}

void DynamicWindowApproach::visualizePath(const std::vector<geometry_msgs::Pose2D::Ptr>& path)
{
    geometry_msgs::PoseArray::Ptr pa(new geometry_msgs::PoseArray());
    pa->poses.resize(path.size());
    pa->header.stamp = ros::Time::now();
    pa->header.frame_id = "map";
    for(size_t i = 0; i < static_cast<size_t>(path.size()); ++i)
    {
        pa->poses[i].position.x = path[i]->x;
        pa->poses[i].position.y = path[i]->y;
        pa->poses[i].orientation.z = std::sin(path[i]->theta / 2);
        pa->poses[i].orientation.w = std::cos(path[i]->theta / 2);
    }
    pub_path_points_.publish(pa);
}

void DynamicWindowApproach::timerListenGoalCallback(const ros::TimerEvent& e)
{
    try
    {
        *goal_ = tf_buffer_.lookupTransform("map", "goal", ros::Time(0));
        goal_flag_ = true;
    }
    catch(const tf2::TransformException& ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }
}

void DynamicWindowApproach::scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    laser_scan_ = msg;
    scan_flag_ = true;
}

void DynamicWindowApproach::timerPlannerCallback(const ros::TimerEvent& e)
{
    geometry_msgs::Pose2D::Ptr pose = createPose();
    
    if(pose)
    {
        goalJudgment(pose);
    }

    if(pose && scan_flag_ && goal_flag_ && !reach_goal_)
    {
        geometry_msgs::TwistStamped::Ptr ts(new geometry_msgs::TwistStamped());
        Window window = createWindow(twist_);
        twist_ = selectBestTwist(pose, twist_, window);
        ROS_INFO(" linear  : %lf", twist_->linear.x);
        ROS_INFO(" angular : %lf", twist_->angular.z);
        ts->header.stamp = ros::Time::now();
        ts->twist = *twist_;
        pub_twist_.publish(ts);
    }
}