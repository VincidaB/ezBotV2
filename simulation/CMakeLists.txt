cmake_minimum_required(VERSION 3.5)
project(ezbot-v2-simulation)


find_package(ament_cmake REQUIRED)


install(
    DIRECTORY launch worlds models
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY models/
    DESTINATION $ENV{HOME}/.gazebo/models/
)


ament_package()