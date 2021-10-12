cmake_minimum_required(VERSION 3.0.2)
project(kino_gripper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  roscpp
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


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
  INCLUDE_DIRS include
  LIBRARIES dynamixel_sdk
  CATKIN_DEPENDS dynamixel_sdk roscpp
#  DEPENDS system_lib
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)



## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/kino_gripper.cpp
   src/${PROJECT_NAME}/dynamixel_ax12_handler.cpp
   src/${PROJECT_NAME}/gripper_inverse_kinematics.cpp
 )

add_executable(dynamixel_cpp_sample src/dynamixel_cpp_sample.cpp)
target_link_libraries(dynamixel_cpp_sample ${catkin_LIBRARIES})

add_executable(kino_gripper_node src/kino_gripper_node.cpp)
target_link_libraries(kino_gripper_node ${PROJECT_NAME} ${catkin_LIBRARIES})
