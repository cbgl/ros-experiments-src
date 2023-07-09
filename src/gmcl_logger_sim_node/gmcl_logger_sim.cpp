#include <gmcl_logger_sim_node/gmcl_logger_sim.h>


/*******************************************************************************
 * Constructor. Initializes params / subscribers.
 */
gmclLogger::gmclLogger(void) :
                       first_ground_truth_time_ (0),
                       num_ground_truths_(0)
{
  loadParams();

  ground_truth_sub_ = nodehandle_.subscribe(ground_truth_topic_, 100,
    &gmclLogger::groundTruthCallback, this);

  global_pose_sub_ = nodehandle_.subscribe(global_pose_topic_, 100,
    &gmclLogger::globalPoseCallback, this);

  map_sub_ = nodehandle_.subscribe(map_topic_, 1,
    &gmclLogger::mapCallback, this);

  initLogfiles();

  sleep(25);

  for (unsigned int i = 0; i < 100; i++)
  {
    callgmcl();
    ros::Duration(6.0).sleep();
  }

  ROS_INFO("[gmclLogger] Node initialised");
}


/*******************************************************************************
 * Destructor
 */
gmclLogger::~gmclLogger(void)
{
  ROS_INFO("[gmclLogger] Node destroyed");
}


/*******************************************************************************
 * Logs execution time
 */
void gmclLogger::executionTime(const ros::Duration& d)
{
  std::ofstream file(execution_time_filename_.c_str(), std::ios::app);
  if (file.is_open())
  {
    file << d << std::endl;
    file.close();
  }
  else
    ROS_ERROR("[gmclLogger] Could not log execution time");
}


/*******************************************************************************
 * Logs the global pose found
 */
void gmclLogger::globalPoseCallback(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  clock_stop_ = ros::Time::now();
  logPose(msg, global_pose_filename_);

  executionTime(clock_stop_-clock_start_);
}


/*******************************************************************************
 * Logs the ground truth pose of the robot.
 */
void gmclLogger::groundTruthCallback(const nav_msgs::Odometry& msg)
{
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
void gmclLogger::initLogfiles(void)
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
    ROS_ERROR("[gmclLogger] Ground truth file not open");

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
    ROS_ERROR("[gmclLogger] global pose file not open");

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
    ROS_ERROR("[gmclLogger] execution-time file not open");

  //----------------------------------------------------------------------------
  // Create the map file
  std::ofstream map_file(map_filename_.c_str());
  if (map_file.is_open())
  {
    //map_file << "occupied_x, occupied_y" << std::endl;
    map_file.close();
  }
  else
    ROS_ERROR("[gmclLogger] Map file not open");
}


/*******************************************************************************
 * Param loader. Check logger.h
 */
void gmclLogger::loadParams(void)
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
  nodehandle_.getParam(ros::this_node::getName() + "/start_signal_service_name",
    start_signal_service_name_);

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
void gmclLogger::logPose(
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
    ROS_ERROR("[gmclLogger] Could not log pose");
}


/*******************************************************************************
 * Logs the map
 */
void gmclLogger::mapCallback(
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
 * Publishes a start signal to gmcl
 */
void gmclLogger::callgmcl()
{
  // Call gmcl service
  ros::ServiceClient client =
    nodehandle_.serviceClient<std_srvs::Empty>(start_signal_service_name_);
  std_srvs::Empty srv;

  while(!client.call(srv))
  {
    ROS_ERROR("[gmclLogger] Failed to call gmcl");
    ros::Duration(0.1).sleep();
  }

  clock_start_ = ros::Time::now();
  ROS_INFO("[gmclLogger] Successful call to gmcl");
}
