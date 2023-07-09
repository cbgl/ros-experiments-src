add_executable(als_bag_player_node
  als_bag_player.cpp
  als_bag_player_node.cpp)

target_link_libraries(als_bag_player_node
  ${catkin_LIBRARIES}
)
