cmake_minimum_required(VERSION 2.8.3)
project(laser_nav)

set(3iRoboticsLidar_SDK_PATH "./sdk/")

FILE(GLOB 3iRoboticsLidar_SDK_SRC 
  "${3iRoboticsLidar_SDK_PATH}/src/*.cpp"
)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  move_base_msgs
  roscpp
  tf
  visualization_msgs
  rosconsole
  sensor_msgs
)


include_directories(
  ${3iRoboticsLidar_SDK_PATH}/include
  ${3iRoboticsLidar_SDK_PATH}/src
  ${catkin_INCLUDE_DIRS}
)


catkin_package()

add_executable(iiiRobotics_lidar_node src/node.cpp ${3iRoboticsLidar_SDK_SRC})
target_link_libraries(iiiRobotics_lidar_node ${catkin_LIBRARIES} -lrt)

add_executable(iiiRobotics_lidar_node_client src/client.cpp)
target_link_libraries(iiiRobotics_lidar_node_client ${catkin_LIBRARIES})

install(TARGETS iiiRobotics_lidar_node iiiRobotics_lidar_node_client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch rviz sdk
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

