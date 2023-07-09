#include <cbgl_bag_player_node/cbgl_bag_player.h>


/*******************************************************************************
 * Constructor. Initializes params / subscribers.
 */
CBGLbag_player::CBGLbag_player(void)
{
  loadParams();

  cbgl_status_sub_ = nodehandle_.subscribe(cbgl_status_topic_, 10,
    &CBGLbag_player::cbglStatusCallback, this);

  ground_truth_pub_ =
    nodehandle_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      ground_truth_topic_,1);
  scan_pub_ =
    nodehandle_.advertise<sensor_msgs::LaserScan>(scan_topic_,1);

  ros::Duration(1.0).sleep();


  ROS_INFO("[CBGLbag_player] Node initialised");

  playBag();
}


/*******************************************************************************
 * Destructor
 */
CBGLbag_player::~CBGLbag_player(void)
{
  ROS_INFO("[CBGLbag_player] Node destroyed");
}


/*******************************************************************************
 * Publishes a start signal to CBGL
 */
void CBGLbag_player::callCBGL()
{
  // Call CBGL service
  ros::ServiceClient client =
    nodehandle_.serviceClient<std_srvs::Empty>(start_signal_service_name_);
  std_srvs::Empty srv;

  while(!client.call(srv))
  {
    ROS_ERROR("[CBGLbag_player] Failed to call CBGL");
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("[CBGLbag_player] Successful call to CBGL");
}


/*******************************************************************************
 * What is CBGL up to? 0 for chilling, > 0 for busy. BS function now that i
 * look at it
 */
void CBGLbag_player::cbglStatusCallback(const std_msgs::UInt64& msg)
{
  if (msg.data == 0)
    cbgl_busy_ = false;
  else
    cbgl_busy_ = true;
}


/*******************************************************************************
 * Param loader. Check bag_player.h
 */
void CBGLbag_player::loadParams(void)
{
  //----------------------------------------------------------------------------
  // Get topic names
  nodehandle_.getParam(ros::this_node::getName() + "/ground_truth_topic",
    ground_truth_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/scan_topic",
    scan_topic_);
  nodehandle_.getParam(ros::this_node::getName() + "/cbgl_status_topic",
    cbgl_status_topic_);

  //----------------------------------------------------------------------------
  // Get bag filename
  nodehandle_.getParam(ros::this_node::getName() + "/bag_filename",
    bag_filename_);
  nodehandle_.getParam(ros::this_node::getName() + "/start_signal_service_name",
    start_signal_service_name_);

  cbgl_busy_ = true;
  num_poses_processed_ = 0;
}


/*******************************************************************************
 * Bagpipe player
 */
void CBGLbag_player::playBag(void)
{
  ROS_INFO("[CBGLbag_player] Playing bag...");
  rosbag::Bag bag;
  bag.open(bag_filename_, rosbag::bagmode::Read);

  std::vector<std::string> topics;
  topics.push_back(std::string(ground_truth_topic_));
  topics.push_back(std::string(scan_topic_));

  rosbag::View view(bag, rosbag::TopicQuery(topics));


  bool enforce_tmax = false;
  int tmax = 26464;
  int t = 0;
  foreach(rosbag::MessageInstance const m, view)
  {
    ROS_INFO("[CBGLbag_player] Call counter is %d", t);
    t++;
    if (enforce_tmax && t < tmax) continue;


    geometry_msgs::PoseWithCovarianceStamped::ConstPtr p =
      m.instantiate<geometry_msgs::PoseWithCovarianceStamped>();

    sensor_msgs::LaserScan::ConstPtr s =
      m.instantiate<sensor_msgs::LaserScan>();

    if (p != NULL)
    {
      // publish pseudo-ground-truth message
      ROS_INFO("[CBGLbag_player] Publishing pose no. %d", p->header.seq);
      ground_truth_pub_.publish(*p);

      // Call global localisation service
      ROS_INFO("[CBGLbag_player] Calling CBGL service (call no. %d)",
        num_poses_processed_);
      callCBGL();

      num_poses_processed_++;
    }


    // Undersample scan for speed of execution
    unsigned int undersample_rate = 4;
    bool do_undersample = !(undersample_rate == 1);
    if (s != NULL)
    {
      ROS_INFO("[CBGLbag_player] Publishing scan no. %d", s->header.seq);
      if (do_undersample)
      {
        std::vector<float> uranges;
        for (unsigned int i = 0; i < s->ranges.size(); i++)
        {
          if (fmod(i,undersample_rate) == 0)
            uranges.push_back(s->ranges[i]);
        }

        // The new scan message
        sensor_msgs::LaserScan s_ = *s;
        s_.ranges = uranges;
        s_.angle_increment = undersample_rate*s->angle_increment;
        scan_pub_.publish(s_);
      }
      else
        scan_pub_.publish(*s);
    }
  }

  bag.close();
}
