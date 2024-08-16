cmake_minimum_required(VERSION 3.5)
project(ezbot-v2)


find_package(ament_cmake REQUIRED)


install(
    DIRECTORY launch description meshes config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY models/
    DESTINATION $ENV{HOME}/.gz/models/
)


ament_package()