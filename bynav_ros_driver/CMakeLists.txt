cmake_minimum_required(VERSION 3.10)
project(bynav_ros_driver)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)

find_package(pluginlib)
find_package(rclcpp REQUIRED) 
find_package(rclcpp_components REQUIRED) 
find_package(std_msgs REQUIRED)
find_package(gps_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(novatel_oem7_msgs REQUIRED)
find_package(Boost REQUIRED)

LIST(APPEND CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib")

find_package(GeographicLib REQUIRED)

if ($ENV{ROS_DISTRO} STREQUAL "humble")
	add_definitions(-DROS_HUMBLE)
endif()

include_directories(
	include
	src
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/src
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/include/novatel/edie           
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/bin/Novatel/api/
	${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/bin/StreamInterface/api/
	${PROJECT_SOURCE_DIR}/thirdparty/eigen
)

add_library(${PROJECT_NAME} SHARED
	src/oem7_receiver_net.cpp
	src/oem7_receiver_port.cpp
	src/oem7_message_decoder.cpp
	src/oem7_message_decoder_lib.cpp
	src/oem7_message_util.cpp
	src/oem7_ros_messages.cpp
	src/oem7_debug_file.cpp
	src/bestpos_handler.cpp
	src/ins_handler.cpp
	src/align_handler.cpp
	src/time_handler.cpp
	src/receiverstatus_handler.cpp
	src/nmea_handler.cpp
	src/odometry_handler.cpp
	src/oem7_imu.cpp
)

## All components are plugins
add_executable(${PROJECT_NAME}_exe
	src/oem7_message_node.cpp
	src/oem7_message_util.cpp
	src/oem7_ros_messages.cpp
	src/message_handler.cpp
	src/tcp_client.cpp
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64" OR CMAKE_SYSTEM_PROCESSOR STREQUAL "armv7l")
  set(COMMON_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/arm64/libCommon.a)
  set(STREAM_INTERFACE_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/arm64/libStreamInterface.a)
  set(NOVATEL_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/arm64/libNovatel.a)
  set(JSON_INTERFACE_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/arm64/libJsonInterface.a)
else()
  set(COMMON_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/x86/libCommon.a)
  set(STREAM_INTERFACE_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/x86/libStreamInterface.a)
  set(NOVATEL_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/x86/libNovatel.a)
  set(JSON_INTERFACE_LIB ${PROJECT_SOURCE_DIR}/thirdparty/novatel_edie/usr/lib/x86/libJsonInterface.a)
endif()

target_link_libraries(${PROJECT_NAME}
	Boost::boost
	${GeographicLib_LIBRARIES}
  ${STREAM_INTERFACE_LIB}
  ${NOVATEL_LIB}
  ${JSON_INTERFACE_LIB}
  ${COMMON_LIB}
)

ament_target_dependencies(${PROJECT_NAME} 
	pluginlib
	std_msgs 
	gps_msgs
	sensor_msgs 
	nmea_msgs 	
	nav_msgs
	tf2_geometry_msgs 
	novatel_oem7_msgs
)

ament_target_dependencies(${PROJECT_NAME}_exe 
	rclcpp
	rclcpp_components
	pluginlib
	novatel_oem7_msgs 
)

install(TARGETS
  ${PROJECT_NAME}_exe
  DESTINATION lib/${PROJECT_NAME}
)

#############
## Install ##
#############

ament_export_dependencies(ament_cmake)
ament_export_include_directories(include)
pluginlib_export_plugin_description_file(${PROJECT_NAME} bynav_ros_driver_plugins.xml)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} 
	ARCHIVE DESTINATION lib 
	LIBRARY DESTINATION lib 
	RUNTIME DESTINATION bin 
)

install(FILES ${PROJECT_NAME}_plugins.xml DESTINATION share)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.yaml")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.py")
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME} FILES_MATCHING PATTERN "*.launch")

ament_package()


