cmake_minimum_required(VERSION 3.5)
project(stcamera_launcher)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcutils REQUIRED)
find_package(rclcpp_components REQUIRED)

find_package(stcamera_components REQUIRED)
find_package(stcamera_msgs REQUIRED)

find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Specify libraries to link a library or executable target against
add_executable(
  ${PROJECT_NAME}
    src/main.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
  stcamera_components/include
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp
  rclcpp_components
  rcutils
  stcamera_components
  stcamera_msgs

  image_transport
  camera_info_manager
)

# installation
install(TARGETS ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
		DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
		DESTINATION share/${PROJECT_NAME}
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

ament_package()
