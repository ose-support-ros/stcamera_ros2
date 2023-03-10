cmake_minimum_required(VERSION 3.5)
project(stcamera_grabber)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(stcamera_msgs REQUIRED)
find_package(stcamera_components REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
find_package(rcl_interfaces REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)

set(TARGET_INC_DIR
)



## Specify libraries to link a library or executable target against
add_executable(
  ${PROJECT_NAME}
  src/grabber.cpp
  src/stcamera_base.cpp
  src/stcamera.cpp
  src/stmanager_base.cpp
  src/stmanager.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  stcamera_msgs
  stcamera_components
  rclcpp
  #rclcpp_action
  rclcpp_components
  rcutils
  sensor_msgs
  camera_info_manager
  image_transport
  ${ament_EXPORTED_LIBRARIES}
  ${ament_EXPORTED_TARGETS}
)


# installation
install(
  TARGETS
    ${PROJECT_NAME}
    DESTINATION bin/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
