cmake_minimum_required(VERSION 3.5.0)
project(crs_area_selection VERSION 0.1.0 LANGUAGES CXX)

add_compile_options(-std=c++14) # Remove

# Find all dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(crs_msgs REQUIRED)


# TODO: Much of this can likely be simplified by correctly using Ament.

# Targets
# Library containing tools for specifying regions and determining inlying points
add_library(${PROJECT_NAME}_area_selection SHARED src/area_selector.cpp src/selection_artist.cpp)
target_link_libraries(${PROJECT_NAME}_area_selection PUBLIC
  ${ament_index_cpp_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${crs_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${shape_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${visualization_msgs_LIBRARIES}
  ${PCL_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_eigen_LIBRARIES})
set_target_properties(${PROJECT_NAME}_area_selection PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_area_selection PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>")
target_include_directories(${PROJECT_NAME}_area_selection SYSTEM PUBLIC
    ${ament_index_cpp_INCLUDE_DIRS}
    ${rclcpp_INCLUDE_DIRS}
    ${crs_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${sensor_msgs_INCLUDE_DIRS}
    ${shape_msgs_INCLUDE_DIRS}
    ${std_srvs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${visualization_msgs_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
    )
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_area_selection)

# Node that runs area selection service
add_executable(${PROJECT_NAME}_area_selection_node src/area_selection_node.cpp)
target_link_libraries(${PROJECT_NAME}_area_selection_node PRIVATE
    ${PROJECT_NAME}_area_selection
    ${rclcpp_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${tf2_eigen_LIBRARIES}
    )
target_compile_options(${PROJECT_NAME}_area_selection_node PRIVATE -Wall -Wextra -Wsuggest-override -Wconversion -Wsign-conversion)
set_target_properties(${PROJECT_NAME}_area_selection_node PROPERTIES CXX_STANDARD 14)
target_include_directories(${PROJECT_NAME}_area_selection_node PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")
target_include_directories(${PROJECT_NAME}_area_selection_node SYSTEM PUBLIC
#  ${PROJECT_NAME}_area_selection_INCLUDE_DIRS
    ${rclcpp_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
)
list (APPEND PACKAGE_LIBRARIES ${PROJECT_NAME}_area_selection_node)

# Install targets
install(TARGETS
    ${PROJECT_NAME}_area_selection
    ${PROJECT_NAME}_area_selection_node
    EXPORT ${PROJECT_NAME}-targets
    DESTINATION lib/${PROJECT_NAME})

install(EXPORT ${PROJECT_NAME}-targets NAMESPACE crs:: DESTINATION lib/cmake/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)

# This was removed to avoid confusion until serialization is added again
#install(FILES config/area_selection_parameters.yaml
#  DESTINATION share/${PROJECT_NAME}/config
#)

# Create cmake config files
#include(CMakePackageConfigHelpers)
#configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
#  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
#  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
#  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

#write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
#  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

#install(FILES
#  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
#  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
#  DESTINATION lib/cmake/${PROJECT_NAME})

#export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)

# Setup Ament
ament_export_dependencies(
    ament_cmake
    ament_index_cpp
    rclcpp
    tf2_ros
    tf2_eigen
    Eigen3
    PCL
    crs_msgs)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_area_selection)
ament_package()


