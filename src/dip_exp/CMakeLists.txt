cmake_minimum_required(VERSION 3.0.2)
project(dip_exp)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3") #c++11
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native --fast-math -ffast-math -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  cv_bridge
  nav_msgs
  sensor_msgs
  mavros_msgs
  serial
)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

set(THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)


include_directories(
          include
          ${catkin_INCLUDE_DIRS}
          ${CMAKE_CURRENT_SOURCE_DIR}/include
  )
catkin_package(
)


add_executable(exp1
  src/exp1.cpp)
target_link_libraries(exp1
  ${THIRD_PARTY_LIBS}
  ${catkin_LIBRARIES}
)
add_executable(exp2
  src/exp2.cpp)
target_link_libraries(exp2
  ${THIRD_PARTY_LIBS}
  ${catkin_LIBRARIES}
)