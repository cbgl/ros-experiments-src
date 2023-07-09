#include <als_logger_sim_node/als_logger_sim.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "als_logger_sim_node");

  alsLogger als_logger;
  ros::spin();
  return 0;
}
