add_executable(cbgl_logger_bag_node
  cbgl_logger_bag.cpp
  cbgl_logger_bag_node.cpp)

target_link_libraries(cbgl_logger_bag_node
  ${catkin_LIBRARIES}
)
