#include <mcl_logger_sim_node/mcl_logger_sim.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mcl_logger_sim_node");

  mclLogger mcl_logger;
  ros::spin();
  return 0;
}
