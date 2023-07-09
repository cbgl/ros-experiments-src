add_executable(als_logger_bag_node
  als_logger_bag.cpp
  als_logger_bag_node.cpp)

target_link_libraries(als_logger_bag_node
  ${catkin_LIBRARIES}
)
