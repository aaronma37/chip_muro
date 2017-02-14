cmake_minimum_required(VERSION 2.8.3)
project(turtlebot_deployment)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  actionlib
  genmsg
)

find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_message_files( DIRECTORY msg FILES PoseWithName.msg )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)






catkin_package(
  INCLUDE_DIRS include
  LIBRARIES turtlebot_deployment
  CATKIN_DEPENDS roscpp cv_bridge opencv2 std_msgs geometry_msgs nav_msgs move_base_msgs actionlib message_runtime
  DEPENDS
)

###########
## Build ##
###########

file(GLOB_RECURSE HEADER_FILES include/*.h)

#
#add_library(agent src/agent.cpp)
#add_dependencies(agent turtlebot_deployment_gencpp)
#target_link_libraries(agent
#  ${catkin_LIBRARIES}
#)

#add_library(VoronoiDiagramGenerator src/VoronoiDiagramGenerator.cpp)
#target_link_libraries(VoronoiDiagramGenerator
#  ${catkin_LIBRARIES}
#)

#add_executable(simple_deployment src/simple_deployment.cpp src/agent.cpp src/VoronoiDiagramGenerator.cpp ${HEADER_FILES})
#add_dependencies(simple_deployment turtlebot_deployment_gencpp)
#target_link_libraries(simple_deployment
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)

#add_executable(inter_robot_communication src/inter_robot_communication.cpp)
#add_dependencies(inter_robot_communication turtlebot_deployment_gencpp)
#target_link_libraries(inter_robot_communication
#  ${catkin_LIBRARIES}
#)

#add_executable(simple_deployment_dummy src/simple_deployment_dummy.cpp src/agent.cpp src/VoronoiDiagramGenerator.cpp ${HEADER_FILES})
#add_dependencies(simple_deployment_dummy turtlebot_deployment_gencpp)
#target_link_libraries(simple_deployment_dummy
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)


#add_executable(PathFollowing src/PathFollowing.cpp)
#add_dependencies(PathFollowing ${catkin_EXPORTED_TARGETS})
#target_link_libraries(PathFollowing
#${catkin_LIBRARIES}
#)

#add_executable(rosUnitsMove src/rosUnitsMove.cpp)
#add_dependencies(rosUnitsMove ${catkin_EXPORTED_TARGETS})
#target_link_libraries(rosUnitsMove
#${catkin_LIBRARIES}
#)


#add_executable(cyclic_pursuit src/cyclic_pursuit.cpp)
#add_dependencies(cyclic_pursuit ${catkin_EXPORTED_TARGETS})
#target_link_libraries(cyclic_pursuit
#${catkin_LIBRARIES}
#)

add_executable(ekf src/ekf.cpp)
add_dependencies(ekf ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf
${catkin_LIBRARIES}
)
add_executable(ekf2 src/ekf2.cpp)
add_dependencies(ekf2 ${catkin_EXPORTED_TARGETS})
target_link_libraries(ekf2
${catkin_LIBRARIES}
)

add_executable(chip_ekf src/chip_ekf.cpp)
add_dependencies(chip_ekf ${catkin_EXPORTED_TARGETS})
target_link_libraries(chip_ekf
${catkin_LIBRARIES}
)

#add_executable(movingCentroid src/movingCentroid.cpp)
#add_dependencies(movingCentroid ${catkin_EXPORTED_TARGETS})
#target_link_libraries(movingCentroid
#${catkin_LIBRARIES}
#)

add_executable(centroidKeyboard src/centroidKeyboard.cpp)
add_dependencies(centroidKeyboard  ${catkin_EXPORTED_TARGETS})
target_link_libraries(centroidKeyboard 
${catkin_LIBRARIES}
)

