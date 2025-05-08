cmake_minimum_required(VERSION 3.8)
project(vo_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)

# Find OpenCV - this should define OpenCV_LIBS and OpenCV_INCLUDE_DIRS
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d)
message(STATUS "DEBUG: OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message(STATUS "DEBUG: OpenCV_LIBS = ${OpenCV_LIBS}") # Check if this is populated!

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(oak_visual_odometry_node src/oak_visual_odometry.cpp)

target_include_directories(oak_visual_odometry_node PUBLIC # Using PUBLIC here is for include dirs, not linking
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
  ${cv_bridge_INCLUDE_DIRS}
)

# Link ROS dependencies - this uses plain linking internally for some targets
ament_target_dependencies(oak_visual_odometry_node
    rclcpp
    sensor_msgs
    nav_msgs
    geometry_msgs
    cv_bridge
    message_filters
    tf2
    tf2_geometry_msgs
)

# Link OpenCV using a PLAIN signature by directly using OpenCV_LIBS
# This MUST come AFTER ament_target_dependencies if ament_target_dependencies uses plain linking first.
# Or, sometimes, it needs to be BEFORE ament_package()
if(NOT OpenCV_LIBS)
  message(FATAL_ERROR "OpenCV_LIBS is not defined after find_package(OpenCV)! Cannot proceed with plain linking for OpenCV.")
endif()

target_link_libraries(oak_visual_odometry_node
  ${OpenCV_LIBS} # This must be a plain list of libraries
)


install(TARGETS
    oak_visual_odometry_node
    DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY include/
  DESTINATION include
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_package()