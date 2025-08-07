cmake_minimum_required(VERSION 3.8)
project(aruco_tag_detection_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rovers_interfaces REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(message_filters REQUIRED)

add_executable(aruco_tag_detector src/aruco_tag_detector.cpp)
target_include_directories(aruco_tag_detector PUBLIC
  $BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include
  $INSTALL_INTERFACE:include/${PROJECT_NAME})

ament_target_dependencies(aruco_tag_detector
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "visualization_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "rovers_interfaces"
  "pcl_ros"
  "pcl_conversions"
  "PCL"
  "yaml-cpp"
  "message_filters"
)

target_link_libraries(aruco_tag_detector ${PCL_LIBRARIES})
target_include_directories(aruco_tag_detector PRIVATE ${PCL_INCLUDE_DIRS})

target_compile_features(aruco_tag_detector PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS aruco_tag_detector
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
  "cv_bridge"
  "image_transport"
  "OpenCV"
  "rovers_interfaces"
  "pcl_ros"
  "pcl_conversions"
  "yaml-cpp"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
