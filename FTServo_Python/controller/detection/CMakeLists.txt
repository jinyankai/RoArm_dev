cmake_minimum_required(VERSION 3.16)
project(object_detection_3d)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# --------- Interface generation ---------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject3D.msg"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# --------- Install Python pkg -----------
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  object_detection_3d/detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
