#ifndef als_bag_player_H
#define als_bag_player_H

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
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

#define foreach BOOST_FOREACH


class alsbag_player
{
  private:
    ros::NodeHandle nodehandle_;

    //

    // List subscribers
    ros::Subscriber als_status_sub_;

    // List publishers
    ros::Publisher ground_truth_pub_;
    ros::Publisher scan_pub_;

    // List subscribed topics
    std::string ground_truth_topic_;
    std::string scan_topic_;
    std::string als_status_topic_;

    // service name
    std::string start_signal_service_name_;

    // List logfile filenames
    std::string bag_filename_;

    // non-zero for als busy
    int als_busy_;
    int num_poses_processed_;

    // Callbacks
    void alsStatusCallback(const std_msgs::UInt64& msg);

    // Init / helpers
    void callals();
    void loadParams();
    void initLogfiles();
    void playBag(void);


  public:
    alsbag_player(void);
    ~alsbag_player(void);

};



#endif //als_bag_player_H
