add_executable(cbgl_bag_player_node
  cbgl_bag_player.cpp
  cbgl_bag_player_node.cpp)

target_link_libraries(cbgl_bag_player_node
  ${catkin_LIBRARIES}
)
