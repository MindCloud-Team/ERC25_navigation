cmake_minimum_required(VERSION 3.5)
project(bbox_display)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED) 
find_package(rclpy REQUIRED)


find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomBBox.msg"
  "msg/BBoxList.msg"
  DEPENDENCIES geometry_msgs
)



ament_python_install_package(${PROJECT_NAME})


install(PROGRAMS
  bbox_display/bbox_to_marker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()

