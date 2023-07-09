#include <cbgl_logger_sim_node/cbgl_logger_sim.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cbgl_logger_sim_node");

  CBGLLogger cbgl_logger;
  ros::spin();
  return 0;
}
