#ifndef gmcl_LOGGER_SIM_H
#define gmcl_LOGGER_SIM_H

#include <cmath>
#include <iostream>
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
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

class gmclLogger
{
  private:
    ros::NodeHandle nodehandle_;

    //
    unsigned int num_ground_truths_;
    int first_ground_truth_time_;

    // List subscribers
    ros::Subscriber ground_truth_sub_;
    ros::Subscriber global_pose_sub_;
    ros::Subscriber execution_time_sub_;
    ros::Subscriber map_sub_;
    ros::Subscriber best_particle_sub_;
    ros::Subscriber all_hypotheses_sub_;
    ros::Subscriber all_hypotheses_caer_sub_;
    ros::Subscriber top_caer_hypotheses_sub_;

    // List subscribed topics
    std::string ground_truth_topic_;
    std::string global_pose_topic_;
    std::string execution_time_topic_;
    std::string map_topic_;
    std::string best_particle_topic_;
    std::string all_hypotheses_topic_;
    std::string all_hypotheses_caer_topic_;
    std::string top_caer_hypotheses_topic_;

    // service name
    std::string start_signal_service_name_;

    // List logfile filenames
    std::string ground_truth_filename_;
    std::string global_pose_filename_;
    std::string execution_time_filename_;
    std::string map_filename_;
    std::string best_particle_filename_;
    std::string all_hypotheses_filename_;
    std::string all_hypotheses_caer_filename_;
    std::string top_caer_hypotheses_filename_;

    std::string map_name_;
    float map_resolution_;
    int map_height_;
    int map_width_;
    int map_origin_x_;
    int map_origin_y_;
    unsigned int num_particleclouds_;
    ros::Time clock_start_;
    ros::Time clock_stop_;

    // Callbacks
    void groundTruthCallback(const nav_msgs::Odometry& msg);
    void globalPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    void mapCallback(const nav_msgs::OccupancyGrid& msg);
    void executionTime(const ros::Duration& d);

    // Init / helpers
    void loadParams();
    void initLogfiles();
    void logPose(const geometry_msgs::PoseWithCovarianceStamped& msg,
      const std::string& filename);
    void callgmcl();


  public:
    gmclLogger(void);
    ~gmclLogger(void);

};



#endif // gmcl_LOGGER_SIM_H
