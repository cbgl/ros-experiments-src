#include <cbgl_logger_bag_node/cbgl_logger_bag.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cbgl_logger_bag_node");

  CBGLlogger_bag cbgl_logger_bag;
  ros::spin();
  return 0;
}
