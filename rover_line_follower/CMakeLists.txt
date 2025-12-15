cmake_minimum_required(VERSION 3.8)
project(rover_line_follower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(camera_viewer src/camera_viewer.cpp)
add_executable(line_follower src/line_follower.cpp)

target_include_directories(camera_viewer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include
)

ament_target_dependencies(camera_viewer
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
  OpenCV
  geometry_msgs
)

target_link_libraries(camera_viewer
  ${OpenCV_LIBRARIES}
)

target_include_directories(line_follower PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  /opt/ros/jazzy/include
)

ament_target_dependencies(line_follower
  rclcpp
  geometry_msgs
)

install(TARGETS
  camera_viewer
  line_follower
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
