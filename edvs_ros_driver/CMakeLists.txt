cmake_minimum_required(VERSION 2.8.3)
project(edvs_ros_driver)

set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  edvs_driver
  dvs_msgs
  std_msgs
  dynamic_reconfigure
  camera_info_manager
  nodelet
)

generate_dynamic_reconfigure_options(
  cfg/EDVS_ROS_Driver.cfg
)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

# since we need eigen and boost search them as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Boost REQUIRED COMPONENTS system thread)

# export the dependencis of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS 
  	include
  CATKIN_DEPENDS 
  	roscpp 
  	edvs_driver
  	dvs_msgs 
  	std_msgs
  	dynamic_reconfigure
  	camera_info_manager
  	nodelet
  DEPENDS 
  	boost
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# make the executable
add_executable(edvs_ros_driver
  src/driver_node.cpp
  src/driver.cpp
)

# make the nodelet into a library
add_library(edvs_ros_driver_nodelet 
  src/driver_nodelet.cpp
  src/driver.cpp
)

# to build the executable we depend on other packets, 
# they need to be build beforehand, especially the messages
add_dependencies(edvs_ros_driver 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_gencf
)

add_dependencies(edvs_ros_driver_nodelet 
  ${catkin_EXPORTED_TARGETS} 
  ${PROJECT_NAME}_gencf
)

# link the executable to the necesarry libs
target_link_libraries(edvs_ros_driver
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# link the executable to the necesarry libs
target_link_libraries(edvs_ros_driver_nodelet
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Install the nodelet library
install(TARGETS edvs_ros_driver_nodelet
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install other support files for installation
install(FILES edvs_ros_driver_nodelet.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
