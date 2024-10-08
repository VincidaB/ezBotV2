cmake_minimum_required(VERSION 3.5)
project(obstacle_detector)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
find_package(Armadillo REQUIRED)
find_package(Boost 1.54.0 REQUIRED system)
ament_auto_find_build_dependencies()

set(TARGET scans_merger)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp
)
target_link_libraries(${TARGET} ${ARMADILLO_LIBRARIES})
rclcpp_components_register_node(${TARGET}
  PLUGIN "obstacle_detector::ScansMerger"
  EXECUTABLE ${TARGET}_node
)

set(TARGET obstacle_extractor)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp
)
target_link_libraries(${TARGET} ${ARMADILLO_LIBRARIES})
rclcpp_components_register_node(${TARGET}
  PLUGIN "obstacle_detector::ObstacleExtractor"
  EXECUTABLE ${TARGET}_node
)

set(TARGET obstacle_tracker)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp
)
target_link_libraries(${TARGET} ${ARMADILLO_LIBRARIES})
rclcpp_components_register_node(${TARGET}
  PLUGIN "obstacle_detector::ObstacleTracker"
  EXECUTABLE ${TARGET}_node
)

set(TARGET obstacle_publisher)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp
)
target_link_libraries(${TARGET} ${ARMADILLO_LIBRARIES})
rclcpp_components_register_node(${TARGET}
  PLUGIN "obstacle_detector::ObstaclePublisher"
  EXECUTABLE ${TARGET}_node
)

set(TARGET position_estimator)
ament_auto_add_library(${TARGET} SHARED
  src/${TARGET}.cpp
)
rclcpp_components_register_node(${TARGET}
  PLUGIN "obstacle_detector::PositionEstimator"
  EXECUTABLE ${TARGET}_node
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
)
