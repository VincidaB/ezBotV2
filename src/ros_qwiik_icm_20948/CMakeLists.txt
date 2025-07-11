cmake_minimum_required(VERSION 3.5)
project(ros_qwiic_icm_20948)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(MSVC)
  add_compile_definitions(
    _USE_MATH_DEFINES
    _WIN32_WINNT=0x0601
  )
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

add_executable(${PROJECT_NAME} 
  src/main.cpp
  src/wiring_for_ros/SPI.cpp
  src/wiring_for_ros/Wire.cpp
  src/wiring_for_ros/Arduino.cpp
  src/wiring_for_ros/i2c.c
  src/SparkFun_ICM-20948_ArduinoLibrary/src/util/ICM_20948_C.c
  src/SparkFun_ICM-20948_ArduinoLibrary/src/ICM_20948.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES
  MAP_IMPORTED_CONFIG_MINSIZEREL Release
  MAP_IMPORTED_CONFIG_RELWITHDEBINFO Release
  )


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED system)

include_directories(
  include/${PROJECT_NAME}
  ${SOURCE_DIR}/include
  src/wiring_for_ros/include
  src/SparkFun_ICM-20948_ArduinoLibrary/src/util/
  src/SparkFun_ICM-20948_ArduinoLibrary/src/
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  sensor_msgs
  Boost)

install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()