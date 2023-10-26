cmake_minimum_required(VERSION 3.8)
project(object_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(SYSTEM 
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(ip_camera_publisher src/ip_camera_publisher.cpp)

ament_target_dependencies(ip_camera_publisher
  rclcpp
  sensor_msgs
  cv_bridge)

target_link_libraries(ip_camera_publisher
  ament_index_cpp::ament_index_cpp
  ${OpenCV_LIBRARIES}
  nlohmann_json::nlohmann_json)

install(TARGETS
  ip_camera_publisher
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  launch
  config
  DESTINATION share/object_detection/
)

ament_python_install_package(${PROJECT_NAME})
 
install(PROGRAMS
  scripts/yolov8_detection.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  doors_and_tables.pt
  config.json
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
