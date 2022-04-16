cmake_minimum_required(VERSION 3.0.2)
project(rasm)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

#this flag was added based on the recommendation from the nvidia people to overcome the deadlock issue 
#refer to issue #16
# opitmization level - optimization for code size and execution time
add_compile_options(-O1)

#set(CMAKE_BUILD_TYPE Debug)
add_compile_options(-O1)

# "undef NDEBUG" to include asserts in release builds
#remove_definitions in cmake is equivalent to undef in make
remove_definitions(-DNDEBUG)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  message_generation
  nav_msgs
  roslib
)
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  triangle.msg
  RASM_PATH_COMMAND_MSG.msg
  RASM_MAP_MSG.msg
  RASM_GOAL_MSG.msg
  ROBOT_POSE_MSG.msg
  RASM_DRIVE_ARC_MSG.msg
)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
   nav_msgs
 )

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
#   INCLUDE_DIRS include
#  LIBRARIES rasm
  CATKIN_DEPENDS roscpp rospy message_runtime nav_msgs roslib
#  DEPENDS system_lib
)

###########
## Build ##
###########
#make the code position independent so that it can be build into a 
#shared library easily. This is equivalent to using the -fPIC option
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_subdirectory(src/core)
add_subdirectory(src/interfaces)

set(ROS_INTERFACE_SRCS
   ${CMAKE_CURRENT_SOURCE_DIR}/src/interfaces/ros_interface/ros_interface.cc
   ${CMAKE_CURRENT_SOURCE_DIR}/src/interfaces/ros_interface/factory.cc
)

add_library(ros_interface ${ROS_INTERFACE_SRCS})

target_include_directories(ros_interface PUBLIC
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/interfaces/ros_interface/
                        ${CMAKE_CURRENT_SOURCE_DIR}/src/core/common/
                        ${catkin_INCLUDE_DIRS}
)

add_dependencies(ros_interface
                        ${catkin_EXPORTED_TARGETS}
                        rasm_viewer_generate_messages_cpp
)

target_link_libraries(ros_interface LINK_PUBLIC
                        common
                        ${catkin_LIBRARIES}
)

add_executable(rasm_viewer_node 
            src/nodes/rasm_viewer_node.cc
            ${ROS_INTERFACE_SRCS}
)

target_include_directories(rasm_viewer_node PUBLIC 
                ${CMAKE_CURRENT_SOURCE_DIR}/src/core/viewer/
                ${catkin_INCLUDE_DIRS}
)

target_link_libraries(rasm_viewer_node LINK_PUBLIC
                rasm_viewer
                ${catkin_LIBRARIES}
)

add_dependencies(rasm_viewer_node
                        ${catkin_EXPORTED_TARGETS}
                        rasm_viewer_generate_messages_cpp
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}

)

