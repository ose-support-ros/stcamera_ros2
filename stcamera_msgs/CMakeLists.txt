cmake_minimum_required(VERSION 3.5)
project(stcamera_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

## Generate messages in the 'msg' folder
set(MSG_FILES
   msg/GenTLInfo.msg
   msg/DeviceConnection.msg
   msg/GenICamEvent.msg
   msg/ElementFloat64.msg
   msg/ElementInt64.msg
   msg/ElementBool.msg
   msg/ElementString.msg
   msg/ErrorInfo.msg
   msg/Event.msg
   msg/Chunk.msg
)

## Generate services in the 'srv' folder
set(SRV_FILES
    srv/GetDeviceList.srv
    srv/GetModuleList.srv
    srv/GetSDKInfo.srv
    srv/ReadNode.srv
    srv/ReadNodeBool.srv
    srv/ReadNodeEnum.srv
    srv/ReadNodeInt.srv
    srv/ReadNodeFloat.srv
    srv/ReadNodePort.srv
    srv/ReadNodeRegister.srv
    srv/ReadNodeRegisterInfo.srv
    srv/ReadNodeString.srv
    srv/WriteNode.srv
    srv/WriteNodeBool.srv
    srv/WriteNodeEnumInt.srv
    srv/WriteNodeEnumStr.srv
    srv/WriteNodeInt.srv
    srv/WriteNodeFloat.srv
    srv/WriteNodePort.srv
    srv/WriteNodeRegister.srv
    srv/WriteNodeString.srv
    srv/EnableChunk.srv
    srv/EnableTrigger.srv
    srv/EnableEventNode.srv
    srv/EnableImageAcquisition.srv
    srv/EnableEventAcquisition.srv
    srv/GetImageAcquisitionStatus.srv
    srv/GetEventAcquisitionStatusList.srv
    srv/GetEventNodeStatusList.srv
    srv/GetChunkList.srv
    srv/GetTriggerList.srv
    srv/GetEnumList.srv
    srv/GetGenICamNodeInfo.srv
    srv/SendSoftTrigger.srv
    srv/ExecuteNode.srv
)

## Generate actions in the 'action' folder
# add_action_files(ACTION_FILES
#   action/Action1.action
#   action/Action2.action
# )

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
	${SRV_FILES}
	DEPENDENCIES builtin_interfaces std_msgs sensor_msgs std_srvs
	)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
