#include <cbgl_logger_sim_node/cbgl_logger_sim.h>


/*******************************************************************************
 * Constructor. Initializes params / subscribers.
 */
CBGLLogger::CBGLLogger(void) :
                       first_ground_truth_time_ (0),
                       num_ground_truths_(0)
{
  loadParams();

  ground_truth_sub_ = nodehandle_.subscribe(ground_truth_topic_, 100,
    &CBGLLogger::groundTruthCallback, this);

  global_pose_sub_ = nodehandle_.subscribe(global_pose_topic_, 100,
    &CBGLLogger::globalPoseCallback, this);

  execution_time_sub_ = nodehandle_.subscribe(execution_time_topic_, 100,
    &CBGLLogger::executionTimeCallback, this);

  map_sub_ = nodehandle_.subscribe(map_topic_, 1,
    &CBGLLogger::mapCallback, this);

  best_particle_sub_ = nodehandle_.subscribe(best_particle_topic_, 100,
    &CBGLLogger::numParticlesCallback, this);

  all_hypotheses_sub_ = nodehandle_.subscribe(all_hypotheses_topic_, 10000000000,
    &CBGLLogger::allHypothesesCallback, this);
  all_hypotheses_caer_sub_ = nodehandle_.subscribe(all_hypotheses_caer_topic_,
    10000000000, &CBGLLogger::allHypothesesCAERCallback, this);
  top_caer_hypotheses_sub_ = nodehandle_.subscribe(top_caer_hypotheses_topic_, 1000,
    &CBGLLogger::topCAERHypothesesCallback, this);

  initLogfiles();

  sleep(25);

  for (unsigned int i = 0; i < 100; i++)
  {
    callCBGL();
    ros::Duration(6.0).sleep();
  }

  ROS_INFO("[CBGLLogger] Node initialised");
}


/*******************************************************************************
 * Destructor
 */
CBGLLogger::~CBGLLogger(void)
{
  ROS_INFO("[CBGLLogger] Node destroyed");
}


/*******************************************************************************
 * Logs the poses of all initial pose hypotheses
 */
void CBGLLogger::allHypothesesCallback(const geometry_msgs::PoseArray& pose_array)
{
  num_particleclouds_++;
  if (num_particleclouds_ != 2)
    return;

  for (unsigned int i = 0; i < pose_array.poses.size(); i++)
  {
    geometry_msgs::PoseWithCovarianceStamped pose;

    pose.header.stamp = pose_array.header.stamp;
    pose.pose.pose = pose_array.poses[i];

    logPose(pose, all_hypotheses_filename_);
  }
}


/*******************************************************************************
 * Logs the CAER initial pose hypotheses
 */
void CBGLLogger::allHypothesesCAERCallback(
  const geometry_msgs::PoseArray& pose_array)
{
  num_particleclouds_++;
  if (num_particleclouds_ != 2)
    return;

  std::ofstream file(all_hypotheses_caer_filename_.c_str(), std::ios::app);
  if (file.is_open())
  {
    for (unsigned int i = 0; i < pose_array.poses.size(); i++)
      file << pose_array.poses[i].position.x << std::endl;

    file.close();
  }
  else
    ROS_ERROR("[CBGLLogger] Could not log CAER");
}


/*******************************************************************************
 * Logs execution time
 */
void CBGLLogger::executionTimeCallback(const std_msgs::Duration& msg)
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
    ROS_ERROR("[CBGLLogger] Could not log execution time");
}


/*******************************************************************************
 * Logs the global pose found
 */
void CBGLLogger::globalPoseCallback(
  const geometry_msgs::PoseWithCovarianceStamped& msg)
{
  logPose(msg, global_pose_filename_);
}


/*******************************************************************************
 * Logs the ground truth pose of the robot.
 */
void CBGLLogger::groundTruthCallback(const nav_msgs::Odometry& msg)
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
void CBGLLogger::initLogfiles(void)
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
    ROS_ERROR("[CBGLLogger] Ground truth file not open");

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
    ROS_ERROR("[CBGLLogger] global pose file not open");

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
    ROS_ERROR("[CBGLLogger] execution-time file not open");

  //----------------------------------------------------------------------------
  // Create the map file
  std::ofstream map_file(map_filename_.c_str());
  if (map_file.is_open())
  {
    //map_file << "occupied_x, occupied_y" << std::endl;
    map_file.close();
  }
  else
    ROS_ERROR("[CBGLLogger] Map file not open");

  //----------------------------------------------------------------------------
  // Create the number of particles file
  std::ofstream best_particle_file(best_particle_filename_.c_str());
  if (best_particle_file.is_open())
  {
    //std::string header = "number of particles";
    //best_particle_file << header << std::endl;
    best_particle_file.close();
  }
  else
    ROS_ERROR("[CBGLLogger] number of particles file not open");

  //----------------------------------------------------------------------------
  // Create all hypotheses file
  std::ofstream all_hypotheses_file(all_hypotheses_filename_.c_str());
  if (all_hypotheses_file.is_open())
  {
    //std::string header = "position.x, position.y, orientation.yaw";
    //all_hypotheses_file << header << std::endl;
    all_hypotheses_file.close();
  }
  else
    ROS_ERROR("[CBGLLogger] all hypotheses file not open");

  //----------------------------------------------------------------------------
  // Create all hypotheses caers file
  std::ofstream all_hypotheses_caer_file(all_hypotheses_caer_filename_.c_str());
  if (all_hypotheses_caer_file.is_open())
  {
    //std::string header = "position.x, position.y, orientation.yaw";
    //all_hypotheses_file << header << std::endl;
    all_hypotheses_caer_file.close();
  }
  else
    ROS_ERROR("[CBGLLogger] all hypotheses caer file not open");

  //----------------------------------------------------------------------------
  // Create top caer hypotheses file
  std::ofstream top_caer_hypotheses_file(top_caer_hypotheses_filename_.c_str());
  if (top_caer_hypotheses_file.is_open())
  {
    //std::string header = "position.x, position.y, orientation.yaw";
    //top_caer_hypotheses_file << header << std::endl;
    top_caer_hypotheses_file.close();
  }
  else
    ROS_ERROR("[CBGLLogger] top caer hypotheses file not open");
}


/*******************************************************************************
 * Param loader. Check logger.h
 */
void CBGLLogger::loadParams(void)
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
  nodehandle_.getParam(ros::this_node::getName() + "/best_particle_topic",
    best_particle_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/start_signal_service_name",
    start_signal_service_name_);
  nodehandle_.getParam(ros::this_node::getName() + "/all_hypotheses_topic",
    all_hypotheses_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/all_hypotheses_caer_topic",
    all_hypotheses_caer_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/top_caer_hypotheses_topic",
    top_caer_hypotheses_topic_);


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
  nodehandle_.getParam(ros::this_node::getName() + "/best_particle_filename",
    best_particle_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/all_hypotheses_filename",
    all_hypotheses_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/all_hypotheses_caer_filename",
    all_hypotheses_caer_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/top_caer_hypotheses_filename",
    top_caer_hypotheses_filename_);

  nodehandle_.getParam(ros::this_node::getName() + "/map_name",
    map_name_);

  num_particleclouds_ = 0;
}


/*******************************************************************************
 * Logs a pose
 */
void CBGLLogger::logPose(
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
    ROS_ERROR("[CBGLLogger] Could not log pose");
}


/*******************************************************************************
 * Logs the map
 */
void CBGLLogger::mapCallback(
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
 * Logs number of particles used
 */
void CBGLLogger::numParticlesCallback(
  const std_msgs::UInt64& msg)
{
  std::ofstream best_particle_file(best_particle_filename_.c_str(), std::ios::app);

  if (best_particle_file.is_open())
  {
    best_particle_file << msg.data << std::endl;
    best_particle_file.close();
  }
}


/*******************************************************************************
 * Logs the poses of the top-caer pose hypotheses
 */
void CBGLLogger::topCAERHypothesesCallback(const geometry_msgs::PoseArray& pose_array)
{
  for (unsigned int i = 0; i < pose_array.poses.size(); i++)
  {
    geometry_msgs::PoseWithCovarianceStamped pose;

    pose.header.stamp = pose_array.header.stamp;
    pose.pose.pose = pose_array.poses[i];

    logPose(pose, top_caer_hypotheses_filename_);
  }
}

/*******************************************************************************
 * Publishes a start signal to CBGL
 */
void CBGLLogger::callCBGL()
{
  // Call CBGL service
  ros::ServiceClient client =
    nodehandle_.serviceClient<std_srvs::Empty>(start_signal_service_name_);
  std_srvs::Empty srv;

  while(!client.call(srv))
  {
    ROS_ERROR("[CBGLLogger] Failed to call CBGL");
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("[CBGLLogger] Successful call to CBGL");
}
