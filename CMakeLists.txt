cmake_minimum_required(VERSION 3.0.2)
project(ais_serial_ros)

# Find catkin macros and libraries =================================
find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

# Generate messages in the 'msg' folder ============================
add_message_files(FILES
   ais_data.msg
)

generate_messages(DEPENDENCIES
   std_msgs
   geometry_msgs
)

# catkin specific configuration =====================================
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES marnav
  CATKIN_DEPENDS
    message_runtime
    roscpp
    serial
    sensor_msgs
    std_msgs
    geometry_msgs
)

# Specify additional locations of header files ======================
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ~/catkin_ws/src/marnav/include/
)

# the last folder depending of the architecture (64, aarch64, ...)
link_directories(~/catkin_ws/src/marnav/build/src)
SET(CMAKE_INSTALL_RPATH ~/catkin_ws/src/marnav/build/src)

# ais_serial_ros_node ===============================================
set(OTHERS_AIS_SOURCES
    src/if_ais_serial.cpp)

add_executable(${PROJECT_NAME}_node
        src/ais_serial_ros_node.cpp
        ${OTHERS_AIS_SOURCES})

add_dependencies(${PROJECT_NAME}_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_node
	${catkin_LIBRARIES}
	marnav
)
