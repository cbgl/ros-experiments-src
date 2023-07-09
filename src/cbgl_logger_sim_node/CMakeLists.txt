add_executable(cbgl_logger_sim_node
  cbgl_logger_sim.cpp
  cbgl_logger_sim_node.cpp)

target_link_libraries(cbgl_logger_sim_node
  ${catkin_LIBRARIES}
)
