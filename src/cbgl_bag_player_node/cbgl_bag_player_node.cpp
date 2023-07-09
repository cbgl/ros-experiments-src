#include <cbgl_bag_player_node/cbgl_bag_player.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "cbgl_bag_player_node");

  CBGLbag_player cbgl_bag_player;
  ros::spin();
  return 0;
}
