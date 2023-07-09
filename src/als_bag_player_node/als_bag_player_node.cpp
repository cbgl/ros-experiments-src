#include <als_bag_player_node/als_bag_player.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "als_bag_player_node");

  alsbag_player als_bag_player;
  ros::spin();
  return 0;
}
