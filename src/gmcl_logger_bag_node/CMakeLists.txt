add_executable(gmcl_logger_bag_node
  gmcl_logger_bag.cpp
  gmcl_logger_bag_node.cpp)

target_link_libraries(gmcl_logger_bag_node
  ${catkin_LIBRARIES}
)
