add_executable(als_logger_sim_node
  als_logger_sim.cpp
  als_logger_sim_node.cpp)

target_link_libraries(als_logger_sim_node
  ${catkin_LIBRARIES}
)
