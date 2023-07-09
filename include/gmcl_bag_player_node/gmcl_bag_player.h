#ifndef gmcl_bag_player_H
#define gmcl_bag_player_H

#include <cmath>
#include <iostream>
#include <fstream>
#include <time.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Duration.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_msgs/TFMessage.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH


class gmclbag_player
{
  private:
    ros::NodeHandle nodehandle_;

    //

    // List subscribers
    ros::Subscriber gmcl_status_sub_;

    // List publishers
    ros::Publisher ground_truth_pub_;
    ros::Publisher scan_pub_;
    ros::Publisher clock_pub_;
    ros::Publisher map_pub_;
    ros::Publisher tf_pub_;
    ros::Publisher tf_static_pub_;

    // List subscribed topics
    std::string ground_truth_topic_;
    std::string scan_topic_;
    std::string gmcl_status_topic_;

    // service name
    std::string start_signal_service_name_;

    // List logfile filenames
    std::string bag_filename_;

    // non-zero for gmcl busy
    int gmcl_busy_;
    int num_poses_processed_;

    // Callbacks
    void gmclStatusCallback(const std_msgs::UInt64& msg);

    // Init / helpers
    void callgmcl();
    void loadParams();
    void initLogfiles();
    void playBag(void);


  public:
    gmclbag_player(void);
    ~gmclbag_player(void);

};



#endif //gmcl_bag_player_H
