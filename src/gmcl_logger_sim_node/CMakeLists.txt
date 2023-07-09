add_executable(gmcl_logger_sim_node
  gmcl_logger_sim.cpp
  gmcl_logger_sim_node.cpp)

target_link_libraries(gmcl_logger_sim_node
  ${catkin_LIBRARIES}
)
