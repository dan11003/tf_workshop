cmake_minimum_required(VERSION 2.8.3)
project(tf_workshop)

set (CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE release)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf_conversions
    tf
    genmsg
    sensor_msgs
    std_msgs
    geometry_msgs
    nav_msgs
    eigen_conversions
    #cmake_modules
    )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tf_workshop
  CATKIN_DEPENDS  roscpp tf_conversions tf genmsg sensor_msgs std_msgs geometry_msgs eigen_conversions message_generation nav_msgs # cmake_modules
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/tf_workshop.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
 add_executable(odom2tfchild src/odom2tfchild.cpp)
target_link_libraries(odom2tfchild ${catkin_LIBRARIES} )
