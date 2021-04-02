cmake_minimum_required(VERSION 2.8.3)
project(profi2021_master_solution)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# Debug option
 ADD_DEFINITIONS(-DDEBUG)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    roslint
    tf
    urdf
    rospy
    roslib
    message_generation
    orocos_kdl
    kdl_parser
    brics_actuator
)
find_package(OpenCV REQUIRED)
find_package(Boost COMPONENTS system thread REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

include_directories (
    src
    ${catkin_INCLUDE_DIRS}
)

# For c++ example
add_executable(master_test src/test.cpp)
target_link_libraries(master_test ${catkin_LIBRARIES})

# For python example
catkin_install_python(PROGRAMS scripts/test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS
    master_test
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
