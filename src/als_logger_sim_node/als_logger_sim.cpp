#include <als_logger_sim_node/als_logger_sim.h>


/*******************************************************************************
 * Constructor. Initializes params / subscribers.
 */
alsLogger::alsLogger(void) :
                       first_ground_truth_time_ (0),
                       num_ground_truths_(0),
                       num_global_poses_(0),
                       do_log_global_pose_(false),
                       do_log_ground_truth_(false),
                       sent_motion_update_(false)
{
  loadParams();

  ground_truth_sub_ = nodehandle_.subscribe(ground_truth_topic_, 1,
    &alsLogger::groundTruthCallback, this);

  global_pose_sub_ = nodehandle_.subscribe("/mcl_pose", 1,
    &alsLogger::globalPoseCallback, this);

  execution_time_sub_ = nodehandle_.subscribe(execution_time_topic_, 100,
    &alsLogger::executionTimeCallback, this);

  map_sub_ = nodehandle_.subscribe(map_topic_, 1,
    &alsLogger::mapCallback, this);

  initialpose_pub_ = nodehandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
    "/initialpose", 1);
  motion_pub_ = nodehandle_.advertise<geometry_msgs::Twist>(
    "/cmd_vel_mux/input/teleop", 1);


  initLogfiles();

  sleep(10);

  callals();

  ROS_INFO("[alsLogger] Node initialised");
}


/*******************************************************************************
 * Destructor
 */
alsLogger::~alsLogger(void)
{
  ROS_INFO("[alsLogger] Node destroyed");
}


/*******************************************************************************
 * Logs execution time
 */
void alsLogger::executionTimeCallback(const std_msgs::Duration& msg)
{
  std::ofstream file(execution_time_filename_.c_str(), std::ios::app);
  if (file.is_open())
  {
    file << msg.data.sec;
    file << ",";
    file << msg.data.nsec << std::endl;
    file.close();
  }
  else
    ROS_ERROR("[alsLogger] Could not log execution time");
}


/*******************************************************************************
 * Logs the global pose found
 */
void alsLogger::globalPoseCallback(const geometry_msgs::PoseStamped& msg)
{
  if (sent_motion_update_ && do_log_global_pose_)
    poses_after_motion_update_.push_back(msg);
  else
    return;


  clock_stop_ = ros::Time::now();
  ros::Duration store_time = clock_stop_-clock_start_;
  if(store_time.toSec() > 4.0)
    sent_motion_update_ = false;
  else
    return;

  do_log_ground_truth_ = true;

  // Thresholds
  double th_x = 0.1;
  double th_y = 0.1;
  double th_t = 0.1;

  // We can now determine the global pose and the execution time
  size_t id = poses_after_motion_update_.size();
  for (unsigned int i = poses_after_motion_update_.size()-1; i > 0 ; i--)
  {
    // We can now determine the global pose and the execution time
    tf::Quaternion qi(
      poses_after_motion_update_[i].pose.orientation.x,
      poses_after_motion_update_[i].pose.orientation.y,
      poses_after_motion_update_[i].pose.orientation.z,
      poses_after_motion_update_[i].pose.orientation.w);
    qi.normalize();
    tf::Matrix3x3 mi(qi);
    double roll_i, pitch_i, yaw_i;
    mi.getRPY(roll_i, pitch_i, yaw_i);

    tf::Quaternion qj(
      poses_after_motion_update_[i-1].pose.orientation.x,
      poses_after_motion_update_[i-1].pose.orientation.y,
      poses_after_motion_update_[i-1].pose.orientation.z,
      poses_after_motion_update_[i-1].pose.orientation.w);
    qj.normalize();
    tf::Matrix3x3 mj(qj);
    double roll_j, pitch_j, yaw_j;
    mj.getRPY(roll_j, pitch_j, yaw_j);




    double dx = fabs(poses_after_motion_update_[i].pose.position.x -
      poses_after_motion_update_[i-1].pose.position.x);
    double dy = fabs(poses_after_motion_update_[i].pose.position.y -
      poses_after_motion_update_[i-1].pose.position.y);
    double dt = fabs(yaw_i-yaw_j);

    ROS_ERROR("(%f,%f,%f)", dx,dy,dt);


    bool cond_x = dx > th_x;
    bool cond_y = dy > th_y;
    bool cond_t = dt > th_t;

    if (cond_x && cond_y && cond_t)
    {
      id = i;
      break;
    }
  }

  if (id != poses_after_motion_update_.size())
  {
    clock_stop_ = poses_after_motion_update_[id].header.stamp;
    ros::Duration exec_time = clock_stop_-clock_start_;
    double et = exec_time.toSec();



    std::ofstream file(execution_time_filename_.c_str(), std::ios::app);
    if (file.is_open())
    {
      file << et << std::endl;
      file.close();
    }
    else
      ROS_ERROR("[alsLogger] Could not log execution time");
  }



  logPose(msg, global_pose_filename_);
}


/*******************************************************************************
 * Logs the ground truth pose of the robot.
 */
void alsLogger::groundTruthCallback(const nav_msgs::Odometry& msg)
{
  if (!do_log_ground_truth_)
    return;

  num_ground_truths_++;
  geometry_msgs::PoseWithCovarianceStamped ground_truth_pose;

  ground_truth_pose.header.stamp = msg.header.stamp;
  ground_truth_pose.pose = msg.pose;

  if (num_ground_truths_ > 1)
    return;


  if (map_name_.compare("fmt_corridor") == 0)
  {
    // resolution: 0.025 m/cell
    //ground_truth_pose.pose.pose.position.x += 12.2;
    //ground_truth_pose.pose.pose.position.y += 8.2;

    // resolution: 0.01 m/cell
    ground_truth_pose.pose.pose.position.x += 11.56;
    ground_truth_pose.pose.pose.position.y += 8.2;

    // resolution: 0.005 m/cell
    //ground_truth_pose.pose.pose.position.x += 11.56;
    //ground_truth_pose.pose.pose.position.y += 8.04;
  }

  if (map_name_.compare("fmt_home_karto") == 0)
  {
    ground_truth_pose.pose.pose.position.x += 17.597106;
    ground_truth_pose.pose.pose.position.y += 4.401109;
  }


  if (map_name_.compare("fmt_home_karto2") == 0)
  {
    ground_truth_pose.pose.pose.position.x += 10.490357;
    ground_truth_pose.pose.pose.position.y += 12.100007;
  }

  if (map_name_.compare("fmt_home_karto3") == 0)
  {
    ground_truth_pose.pose.pose.position.x += 20.440000;
    ground_truth_pose.pose.pose.position.y += 16.040000;
  }

  if (map_name_.compare("fmt_willowgarage") == 0)
  {
    ground_truth_pose.pose.pose.position.x += 60.2;
    ground_truth_pose.pose.pose.position.y += 55.9;
  }

  if (map_name_.compare("fmt_warehouse_karto") == 0)
  {
    ground_truth_pose.pose.pose.position.x += 2.083190;
    ground_truth_pose.pose.pose.position.y += 3.015576;
  }

  if (map_name_.compare("fmt_dirtrack") == 0)
  {
    ground_truth_pose.pose.pose.position.x += 3.346388;
    ground_truth_pose.pose.pose.position.y += 17.199677;
  }

  logPose(ground_truth_pose, ground_truth_filename_);
}


/*******************************************************************************
 * Create the logfiles and register the headers.
 */
void alsLogger::initLogfiles(void)
{
  //----------------------------------------------------------------------------
  // Create the ground truth file
  std::ofstream ground_truth_file(ground_truth_filename_.c_str());
  if (ground_truth_file.is_open())
  {
    //std::string header = "position.x, position.y, orientation.yaw";
    //ground_truth_file << header << std::endl;
    ground_truth_file.close();
  }
  else
    ROS_ERROR("[alsLogger] Ground truth file not open");

  //----------------------------------------------------------------------------
  // Create the global pose file
  std::ofstream global_pose_file(global_pose_filename_.c_str());
  if (global_pose_file.is_open())
  {
    //std::string header = "position.x, position.y, orientation.yaw";
    //global_pose_file << header << std::endl;
    global_pose_file.close();
  }
  else
    ROS_ERROR("[alsLogger] global pose file not open");

  //----------------------------------------------------------------------------
  // Create the execution-time file
  std::ofstream execution_time_file(execution_time_filename_.c_str());
  if (execution_time_file.is_open())
  {
    //std::string header = "exec_sec, exec_nsec";
    //execution_time_file << header << std::endl;
    execution_time_file.close();
  }
  else
    ROS_ERROR("[alsLogger] execution-time file not open");

  //----------------------------------------------------------------------------
  // Create the map file
  std::ofstream map_file(map_filename_.c_str());
  if (map_file.is_open())
  {
    //map_file << "occupied_x, occupied_y" << std::endl;
    map_file.close();
  }
  else
    ROS_ERROR("[alsLogger] Map file not open");
}


/*******************************************************************************
 * Param loader. Check logger.h
 */
void alsLogger::loadParams(void)
{
  //----------------------------------------------------------------------------
  // Get topic names
  nodehandle_.getParam(ros::this_node::getName() + "/ground_truth_topic",
    ground_truth_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/global_pose_topic",
    global_pose_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/execution_time_topic",
    execution_time_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/map_topic",
    map_topic_);


  //----------------------------------------------------------------------------
  // Get log filenames
  nodehandle_.getParam(ros::this_node::getName() + "/ground_truth_filename",
    ground_truth_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/global_pose_filename",
    global_pose_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/execution_time_filename",
    execution_time_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/map_filename",
    map_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/map_name",
    map_name_);
}


/*******************************************************************************
 * Logs a pose
 */
void alsLogger::logPose(
  const geometry_msgs::PoseWithCovarianceStamped& msg,
  const std::string& filename)
{
  std::ofstream file(filename.c_str(), std::ios::app);

  tf::Quaternion q(
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w);

  q.normalize();

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  if (file.is_open())
  {
    file << msg.pose.pose.position.x;
    file << ", ";
    file << msg.pose.pose.position.y;
    file << ", ";
    file << yaw << std::endl;

    file.close();
  }
  else
    ROS_ERROR("[alsLogger] Could not log pose");
}


/*******************************************************************************
 * Logs a pose
 */
void alsLogger::logPose(
  const geometry_msgs::PoseStamped& msg,
  const std::string& filename)
{
  std::ofstream file(filename.c_str(), std::ios::app);

  tf::Quaternion q(
    msg.pose.orientation.x,
    msg.pose.orientation.y,
    msg.pose.orientation.z,
    msg.pose.orientation.w);

  q.normalize();

  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  if (file.is_open())
  {
    file << msg.pose.position.x;
    file << ", ";
    file << msg.pose.position.y;
    file << ", ";
    file << yaw << std::endl;

    file.close();
  }
  else
    ROS_ERROR("[alsLogger] Could not log pose");
}


/*******************************************************************************
 * Logs the map
 */
void alsLogger::mapCallback(
  const nav_msgs::OccupancyGrid& msg)
{
  std::ofstream map_file(map_filename_.c_str(), std::ios::app);

  if (map_file.is_open())
  {
    map_resolution_ = msg.info.resolution;
    map_height_ = msg.info.height;
    map_width_ = msg.info.width;
    map_origin_x_ = msg.info.origin.position.x;
    map_origin_y_ = msg.info.origin.position.y;

    for (int i = 0; i < map_height_; i++)
    {
      for (int j = 0; j < map_width_; j++)
      {
        unsigned int id = i * map_width_ + j;
        int cell = msg.data[id];

        if (cell == 100)
        {
          map_file << map_origin_x_ + j * map_resolution_;
          map_file << ", ";
          map_file << map_origin_y_ + i * map_resolution_;
          map_file << std::endl;
        }
      }
    }

    map_file.close();
  }
}



/*******************************************************************************
 * Publishes a start signal to als
 */
void alsLogger::callals()
{
  // Send the robot somewhere within the free space
  geometry_msgs::PoseWithCovarianceStamped initial_pose;

  if (map_name_.compare("fmt_warehouse_karto") == 0)
  {
    initial_pose.pose.pose.position.x = 2;
    initial_pose.pose.pose.position.y = 18;
    initial_pose.pose.pose.position.z = 0;
    initial_pose.pose.pose.orientation.x = 0;
    initial_pose.pose.pose.orientation.y = 0;
    initial_pose.pose.pose.orientation.z = 0;
    initial_pose.pose.pose.orientation.w = 1;
  }

  if (map_name_.compare("fmt_willowgarage") == 0)
  {
    initial_pose.pose.pose.position.x = 50;
    initial_pose.pose.pose.position.y = 70;
    initial_pose.pose.pose.position.z = 0;
    initial_pose.pose.pose.orientation.x = 0;
    initial_pose.pose.pose.orientation.y = 0;
    initial_pose.pose.pose.orientation.z = 0;
    initial_pose.pose.pose.orientation.w = 1;
  }

  initial_pose.header.stamp = ros::Time::now();
  initialpose_pub_.publish(initial_pose);

  ros::Duration(1.0).sleep();

  // Now move it move it
  geometry_msgs::Twist twist_msg_p;
  twist_msg_p.linear.x = 0.0;
  twist_msg_p.linear.y = 0.0;
  twist_msg_p.linear.z = 0.0;
  twist_msg_p.angular.x = 0.0;
  twist_msg_p.angular.y = 0.0;
  twist_msg_p.angular.z = 0.5;

  // Send motion to robot. Only after this can we log the global pose, the
  // ground truth, and the execution time
  motion_pub_.publish(twist_msg_p);
  sent_motion_update_ = true;
  ROS_ERROR("[alsLogger] sent motion");
  clock_start_ = ros::Time::now();

  ROS_ERROR("[alsLogger] going to log now");
  do_log_global_pose_ = true;


  ROS_INFO("[alsLogger] Successful call to als");
}
