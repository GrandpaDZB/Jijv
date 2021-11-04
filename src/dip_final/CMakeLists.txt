cmake_minimum_required(VERSION 3.0.2)
project(dip_final)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0 -g") #c++11
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
  image_transport
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
          ${CMAKE_CURRENT_SOURCE_DIR}/include/**
)
catkin_package(
)

add_executable(image_publisher
  src/image_publisher.cpp)
target_link_libraries(image_publisher
  ${THIRD_PARTY_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(main_controller
  src/main_controller.cpp)
target_link_libraries(main_controller
  ${THIRD_PARTY_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(target_tracker
  src/target_tracker.cpp)
target_link_libraries(target_tracker
  ${THIRD_PARTY_LIBS}
  ${catkin_LIBRARIES}
)

add_executable(joy_controller
  src/joy_controller.cpp)
target_link_libraries(joy_controller
  ${THIRD_PARTY_LIBS}
  ${catkin_LIBRARIES}
)
