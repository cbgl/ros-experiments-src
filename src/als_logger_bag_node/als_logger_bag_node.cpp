#include <als_logger_bag_node/als_logger_bag.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "als_logger_bag_node");

  alslogger_bag als_logger_bag;
  ros::spin();
  return 0;
}
