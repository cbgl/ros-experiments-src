#ifndef als_LOGGER_SIM_H
#define als_LOGGER_SIM_H

#include <cmath>
#include <iostream>
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Duration.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class alsLogger
{
  private:
    ros::NodeHandle nodehandle_;

    //
    unsigned int num_ground_truths_;
    unsigned int num_global_poses_;
    bool do_log_global_pose_;
    bool do_log_ground_truth_;
    bool sent_motion_update_;
    std::vector<geometry_msgs::PoseStamped> poses_after_motion_update_;
    int first_ground_truth_time_;

    ros::Time clock_start_;
    ros::Time clock_stop_;

    // List subscribers
    ros::Subscriber ground_truth_sub_;
    ros::Subscriber global_pose_sub_;
    ros::Subscriber execution_time_sub_;
    ros::Subscriber map_sub_;
    ros::Publisher initialpose_pub_;
    ros::Publisher motion_pub_;

    // List subscribed topics
    std::string ground_truth_topic_;
    std::string global_pose_topic_;
    std::string execution_time_topic_;
    std::string map_topic_;

    // List logfile filenames
    std::string ground_truth_filename_;
    std::string global_pose_filename_;
    std::string execution_time_filename_;
    std::string map_filename_;

    std::string map_name_;
    float map_resolution_;
    int map_height_;
    int map_width_;
    int map_origin_x_;
    int map_origin_y_;

    // Callbacks
    void groundTruthCallback(const nav_msgs::Odometry& msg);
    void globalPoseCallback(const geometry_msgs::PoseStamped& msg);
    void executionTimeCallback(const std_msgs::Duration& msg);
    void mapCallback(const nav_msgs::OccupancyGrid& msg);

    // Init / helpers
    void loadParams();
    void initLogfiles();
    void logPose(const geometry_msgs::PoseWithCovarianceStamped& msg,
      const std::string& filename);
    void logPose(const geometry_msgs::PoseStamped& msg,
      const std::string& filename);
    void callals();


  public:
    alsLogger(void);
    ~alsLogger(void);

};



#endif // als_LOGGER_SIM_H
