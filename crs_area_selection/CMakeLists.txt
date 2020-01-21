cmake_minimum_required(VERSION 3.5.0)
project(crs_area_selection VERSION 0.1.0 LANGUAGES CXX)



add_compile_options(-std=c++14)

#find_package(catkin REQUIRED COMPONENTS
#  eigen_conversions
#  geometry_msgs
#  opp_msgs_serialization
#  roscpp
#  roslib
#  opp_msgs
#  sensor_msgs
#  std_srvs
#  tf
#  tf_conversions
#  visualization_msgs
#)

find_package(rclcpp REQUIRED)
find_package(crs_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

#catkin_package(
#  INCLUDE_DIRS
#    include
#  LIBRARIES
#    area_selection
#  CATKIN_DEPENDS
#    eigen_conversions
#    geometry_msgs
#    opp_msgs_serialization
#    roscpp
#    roslib
#    opp_msgs
#    sensor_msgs
#    std_srvs
#    tf
#    tf_conversions
#    visualization_msgs
#  DEPENDS
#    EIGEN3
#    PCL
#)



#include_directories(
#  include
#  ${catkin_INCLUDE_DIRS}
#  ${EIGEN3_INCLUDE_DIRS}
#  ${PCL_INCLUDE_DIRS}
#)

# Library containing tools for specifying regions
# and determining inlying points
add_library(${PROJECT_NAME}_area_selection SHARED   src/selection_artist.cpp)
target_link_libraries(${PROJECT_NAME}_area_selection PUBLIC ${rclcpp_LIBRARIES} ${crs_msgs_LIBRARIES} ${PCL_LIBRARIES})
set_target_properties(${PROJECT_NAME}_area_selection PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_area_selection PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_area_selection SYSTEM PUBLIC
    ${rclcpp_INCLUDE_DIRS}
    ${crs_msgs_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
#    ${tf2_ros_INCLUDE_DIRS}
    )
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_area_selection)


## Node that runs area selection service
#add_executable(area_selection_node
#  src/area_selection_node.cpp
#)
#target_link_libraries(area_selection_node
#  area_selection
#  ${catkin_LIBRARIES}
#)
#add_dependencies(area_selection_node
#  ${catkin_EXPORTED_TARGETS}
#  ${${PROJECT_NAME}_EXPORTED_TARGETS}
#)


# Install built targets
install(TARGETS
    ${PROJECT_NAME}_area_selection
#    area_selection_node
    DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

install(FILES config/area_selection_parameters.yaml
  DESTINATION share/${PROJECT_NAME}/config
)
