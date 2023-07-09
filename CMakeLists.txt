cmake_minimum_required(VERSION 2.8.3)
project(cbgl_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -Ofast)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  message_generation
  pcl_ros
)

find_package(PkgConfig)
pkg_check_modules(csm REQUIRED csm)

find_package(CGAL REQUIRED)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #LIBRARIES cbgl
#  LIBRARIES relief_devel
  CATKIN_DEPENDS
  roscpp
  geometry_msgs
  nav_msgs
  message_runtime
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${csm_INCLUDE_DIRS}
  /usr/include/pcl-1.7/
  /usr/include/eigen3/
  #/usr/include/python2.7/
  #/usr/include/octave-4.0.0/octave/
)

link_directories(${csm_LIBRARY_DIRS})

## Declare a C++ library
add_library(cbgl_utils_lib
  src/utils/occupancy_grid_utils/coordinate_conversions.cpp
  src/utils/occupancy_grid_utils/ray_tracer.cpp
)

target_link_libraries(cbgl_utils_lib
  ${catkin_LIBRARIES}
)

add_subdirectory(src/cbgl_node)
add_subdirectory(src/cbgl_logger_sim_node)
add_subdirectory(src/cbgl_logger_bag_node)
add_subdirectory(src/cbgl_bag_player_node)

# als
add_subdirectory(src/als_logger_sim_node)
add_subdirectory(src/als_bag_player_node)
add_subdirectory(src/als_logger_bag_node)

# mcl
add_subdirectory(src/mcl_logger_sim_node)

# gmcl
add_subdirectory(src/gmcl_logger_sim_node)
add_subdirectory(src/gmcl_logger_bag_node)
