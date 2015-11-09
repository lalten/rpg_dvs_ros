cmake_minimum_required(VERSION 2.8.3)
project(edvs_driver)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

# search for everything we need to build the package
find_package(catkin)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# since we need eigen and boost search them as well
find_package(Boost REQUIRED COMPONENTS thread)

include(FindPkgConfig)
pkg_check_modules(LIBUSB1 REQUIRED libusb-1.0)

# export the dependencis of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include ${LIBUSB1_INCLUDE_DIRS}
  DEPENDS libusb-1.0
  LIBRARIES edvs_driver
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${LIBUSB1_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# make the library
add_library(edvs_driver
  src/edvs_driver.cpp
)

target_link_libraries(edvs_driver
  ${LIBUSB1_LIBRARIES}
  ${Boost_LIBRARIES}
)

install(TARGETS edvs_driver
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
