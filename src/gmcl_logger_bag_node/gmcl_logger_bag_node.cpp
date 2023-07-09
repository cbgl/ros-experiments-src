#include <gmcl_logger_bag_node/gmcl_logger_bag.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gmcl_logger_bag_node");

  gmcllogger_bag gmcl_logger_bag;
  ros::spin();
  return 0;
}
