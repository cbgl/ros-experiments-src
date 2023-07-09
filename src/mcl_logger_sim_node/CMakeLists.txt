add_executable(mcl_logger_sim_node
  mcl_logger_sim.cpp
  mcl_logger_sim_node.cpp)

target_link_libraries(mcl_logger_sim_node
  ${catkin_LIBRARIES}
)
