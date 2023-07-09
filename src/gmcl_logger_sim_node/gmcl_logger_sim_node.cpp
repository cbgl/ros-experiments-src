#include <gmcl_logger_sim_node/gmcl_logger_sim.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "gmcl_logger_sim_node");

  gmclLogger gmcl_logger;
  ros::spin();
  return 0;
}
