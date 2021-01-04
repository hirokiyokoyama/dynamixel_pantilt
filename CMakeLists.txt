cmake_minimum_required(VERSION 3.5)
project(dynamixel_pantilt)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  nodes/camera_image_publisher.py
  nodes/dynamixel_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  meshes
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
