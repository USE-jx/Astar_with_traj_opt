cmake_minimum_required(VERSION 3.0.2)
project(simple_optimal_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(casadi REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  costmap_2d
  tf2_ros
  tf2
  path_searcher
)


catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES simple_optimal_control
  CATKIN_DEPENDS roscpp std_msgs costmap_2d tf2 tf2_ros path_searcher
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)
include_directories(/usr/local/include)
link_directories(/usr/local/lib)

## Declare a C++ library
#  add_library(simple_optimal_control
#    src/simple_optimal_control.cpp
#  )
add_executable(simple_optimal_control src/point_motion.cpp src/test_with_init.cpp)


## Specify libraries to link a library or executable target against
 target_link_libraries(simple_optimal_control
   ${catkin_LIBRARIES}
   casadi
   path_searcher
 )

 find_package(PythonLibs 2.7)
 target_include_directories(simple_optimal_control PRIVATE ${PYTHON_INCLUDE_DIRS})
 target_link_libraries(simple_optimal_control ${PYTHON_LIBRARIES})
