cmake_minimum_required(VERSION 3.5)
project(remote_calc)

# find ament and the Python- and ROS-middleware pieces
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2_ros REQUIRED)

# this macro hooks into setup.py for all the Python installs
ament_python_install_package(${PROJECT_NAME})

# ensure your launch/, description/ and config/ dirs get installed
install(
  DIRECTORY launch description config
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    remote_calc/aruco_serial_tf_broadcaster.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
