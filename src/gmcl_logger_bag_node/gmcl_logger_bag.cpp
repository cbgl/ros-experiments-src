#include <gmcl_logger_bag_node/gmcl_logger_bag.h>


/*******************************************************************************
 * Constructor. Initializes params / subscribers.
 */
gmcllogger_bag::gmcllogger_bag(void) : num_ground_truths_(0)
{
  loadParams();

  ground_truth_sub_ = nodehandle_.subscribe(ground_truth_topic_, 10,
    &gmcllogger_bag::groundTruthCallback, this);

  global_pose_sub_ = nodehandle_.subscribe(global_pose_topic_, 10,
    &gmcllogger_bag::globalPoseCallback, this);

  execution_time_sub_ = nodehandle_.subscribe(execution_time_topic_, 10,
    &gmcllogger_bag::executionTimeCallback, this);

  map_sub_ = nodehandle_.subscribe(map_topic_, 1,
    &gmcllogger_bag::mapCallback, this);

  initLogfiles();

  ROS_INFO("[gmcllogger_bag] Node initialised");
}


/*******************************************************************************
 * Destructor
 */
gmcllogger_bag::~gmcllogger_bag(void)
{
  ROS_INFO("[gmcllogger_bag] Node destroyed");
}


/*******************************************************************************
 * Logs execution time
 */
void gmcllogger_bag::executionTimeCallback(const std_msgs::Duration& msg)
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
    ROS_ERROR("[gmcllogger_bag] Could not log execution time");
}


/*******************************************************************************
 * Logs the global pose found
 */
void gmcllogger_bag::globalPoseCallback(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  logPose(msg, global_pose_filename_);
}


/*******************************************************************************
 * Logs the ground truth pose of the robot.
 */
void gmcllogger_bag::groundTruthCallback(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  num_ground_truths_++;

  // First ground truth pose lacks a scan to be paired with
  if (num_ground_truths_ > 1)
    logPose(msg, ground_truth_filename_);
}


/*******************************************************************************
 * Create the logfiles and register the headers.
 */
void gmcllogger_bag::initLogfiles(void)
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
    ROS_ERROR("[gmcllogger_bag] Ground truth file not open");

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
    ROS_ERROR("[gmcllogger_bag] global pose file not open");

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
    ROS_ERROR("[gmcllogger_bag] execution-time file not open");

  //----------------------------------------------------------------------------
  // Create the map file
  std::ofstream map_file(map_filename_.c_str());
  if (map_file.is_open())
  {
    //map_file << "occupied_x, occupied_y" << std::endl;
    map_file.close();
  }
  else
    ROS_ERROR("[gmcllogger_bag] Map file not open");
}


/*******************************************************************************
 * Param loader. Check logger_bag.h
 */
void gmcllogger_bag::loadParams(void)
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

  num_particleclouds_ = 0;
}


/*******************************************************************************
 * Logs a pose
 */
void gmcllogger_bag::logPose(
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
    ROS_ERROR("[gmcllogger_bag] Could not log pose");
}


/*******************************************************************************
 * Logs the map
 */
void gmcllogger_bag::mapCallback(
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
