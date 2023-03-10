cmake_minimum_required(VERSION 3.5)
project(stcamera_components)

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

find_package(stcamera_msgs REQUIRED)

find_package(rcl_interfaces REQUIRED)
#find_package(rclcpp_action REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)


## Specify additional locations of header files
## Your package locations should be listed before other locations

if(WIN32)
link_directories($ENV{STAPI_ROOT_PATH}/lib/x64)
else()
link_directories($ENV{STAPI_ROOT_PATH}/lib)
link_directories($ENV{STAPI_ROOT_PATH}/lib/GenICam)
endif()

## Specify libraries to link a library or executable target against
add_library(${PROJECT_NAME} SHARED
  src/stcamera_node.cpp
  src/stcamera_node_impl.cpp
  src/stparameter.cpp
  src/stcamera_interface.cpp
  src/stcamera_interface_impl.cpp
)

target_include_directories(${PROJECT_NAME}
	PUBLIC
  $ENV{STAPI_ROOT_PATH}/include/GenICam
  $ENV{STAPI_ROOT_PATH}/include/StApi
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

if(WIN32)
  target_link_libraries(${PROJECT_NAME}
    ${ament_LIBRARIES}
    GCBase_MD_VC141_v3_4_Sentech.lib GenApi_MD_VC141_v3_4_Sentech.lib StAPi_TL_MD_VC141_v1_2.lib StAPi_IP_MD_VC141_v1_2.lib
  )
else()
  target_link_libraries(${PROJECT_NAME}
    ${ament_LIBRARIES}
    libGCBase.so libGenApi.so libStApi_TL.so libStApi_IP.so -lpthread
  )
endif()

ament_target_dependencies(${PROJECT_NAME}
  stcamera_msgs
  rclcpp
  #rclcpp_action
  rclcpp_components
  rcutils
  sensor_msgs
  camera_info_manager
  image_transport
  ${ament_EXPORTED_LIBRARIES}
  ${ament_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "stcamera::StCameraNode"
)


# installation
install(
  TARGETS
    ${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION  lib/${PROJECT_NAME}
  )
install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include/${PROJECT_NAME}/
  FILES_MATCHING
    PATTERN "*.hpp"
    PATTERN "*.h"
  	PATTERN "impl" EXCLUDE
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

ament_export_libraries(${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime)
ament_package()
