cmake_minimum_required(VERSION 2.8.3)
project(test_pkg)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  actionlib
  genmsg
  rospy
  message_generation
)

#catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coconuts_odroid
#  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs roscpp rospy serial std_msgs tf cv_bridge image_transport
#  DEPENDS system_lib
CATKIN_DEPENDS message_runtime
)

set(ALL_LIBS
	${catkin_LIBRARIES}
	${OPENGL_LIBRARY}
	glfw
        libglew1.6	
)

add_definitions(
	-DTW_STATIC
	-DTW_NO_LIB_PRAGMA
	-DTW_NO_DIRECT3D
	-DGLEW_STATIC
	-D_CRT_SECURE_NO_WARNINGS
)

find_package(GLUT REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
	common
	external/glfw-3.1.2/include/GLFW/
	external/glm-0.9.7.1/
	external/glew-1.13.0/include/
	external/assimp-3.0.1270/include/
	external/bullet-2.81-rev2613/src/
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OPENGL_INCLUDE_DIRS}
)

#add_message_files( DIRECTORY msg FILES  pathData.msg )
                                        #PoseWithName.msg

#generate_messages(
#  DEPENDENCIES
#  std_msgs
#  geometry_msgs
#)

catkin_package(
  #INCLUDE_DIRS include
  LIBRARIES turtlebot_deployment test_pkg
  CATKIN_DEPENDS roscpp rospy cv_bridge opencv2 std_msgs geometry_msgs nav_msgs move_base_msgs actionlib message_runtime
  DEPENDS
)



###########
## Build ##
###########

file(GLOB_RECURSE HEADER_FILES include/*.h)

#add_executable(simple_deployment src/simple_deployment.cpp src/agent.cpp src/VoronoiDiagramGenerator.cpp ${HEADER_FILES})
#add_dependencies(simple_deployment turtlebot_deployment_gencpp)
#target_link_libraries(simple_deployment
#  ${OpenCV_LIBS}
#  ${catkin_LIBRARIES}
#)

  catkin_install_python(PROGRAMS
    src/formation_stabilization.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

add_executable(quadcopter_ekf src/quadcopter_ekf.cpp)
add_dependencies(quadcopter_ekf ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadcopter_ekf
${catkin_LIBRARIES}
)

add_executable(sample src/sample.cpp)
add_dependencies(sample ${catkin_EXPORTED_TARGETS})
target_link_libraries(sample
${catkin_LIBRARIES}
)

add_executable(slider src/slider.cpp)
add_dependencies(slider ${catkin_EXPORTED_TARGETS})
target_link_libraries(slider
${catkin_LIBRARIES}
)
add_executable(movementTest src/movementTest.cpp)
add_dependencies(movementTest ${catkin_EXPORTED_TARGETS})
target_link_libraries(movementTest
${catkin_LIBRARIES}
)

add_executable(hover src/hover.cpp)
add_dependencies(hover ${catkin_EXPORTED_TARGETS})
target_link_libraries(hover
${catkin_LIBRARIES}
)

add_executable(flight src/flight.cpp)
add_dependencies(flight ${catkin_EXPORTED_TARGETS})
target_link_libraries(flight
${catkin_LIBRARIES}
)

add_executable(quadcopter_ekf_multiple src/quadcopter_ekf_multiple.cpp)
add_dependencies(quadcopter_ekf_multiple ${catkin_EXPORTED_TARGETS})
target_link_libraries(quadcopter_ekf_multiple
${catkin_LIBRARIES}
)



add_executable(path_follower src/path_follower.cpp)
add_dependencies(path_follower ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower
${catkin_LIBRARIES}
)

add_executable(path_follower_test_copy src/path_follower_test_copy.cpp)
add_dependencies(path_follower_test_copy ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower_test_copy
${catkin_LIBRARIES}
)

add_executable(path_follower_test_sample_input src/path_follower_test_sample_input.cpp)
add_dependencies(path_follower_test_sample_input ${catkin_EXPORTED_TARGETS})
target_link_libraries(path_follower_test_sample_input
${catkin_LIBRARIES}
)

add_executable(slider_multiple src/slider_multiple.cpp)
add_dependencies(slider_multiple ${catkin_EXPORTED_TARGETS})
target_link_libraries(slider_multiple
${catkin_LIBRARIES}
)

add_executable(deserializeForAndroid src/deserializeForAndroid.cpp)
add_dependencies(deserializeForAndroid ${catkin_EXPORTED_TARGETS})
target_link_libraries(deserializeForAndroid
${catkin_LIBRARIES}
)

add_executable(collisionAvoidance src/collisionAvoidance.cpp)
add_dependencies(collisionAvoidance ${catkin_EXPORTED_TARGETS})
target_link_libraries(collisionAvoidance
${catkin_LIBRARIES}
)

add_executable(centroidGeneratorRos src/centroidGeneratorRos.cpp src/VoronoiDiagramGenerator.cpp)
add_dependencies(centroidGeneratorRos ${catkin_EXPORTED_TARGETS})
target_link_libraries(centroidGeneratorRos
${catkin_LIBRARIES}
)

add_executable(CoMGeneratorRos src/CoMGeneratorRos.cpp src/VoronoiDiagramGenerator.cpp)
add_dependencies(CoMGeneratorRos ${catkin_EXPORTED_TARGETS})
target_link_libraries(CoMGeneratorRos
${catkin_LIBRARIES}
)

add_executable(CGR src/CGR.cpp src/VoronoiDiagramGenerator.cpp)
add_dependencies(CGR ${catkin_EXPORTED_TARGETS})
target_link_libraries(CGR
${catkin_LIBRARIES}
)

add_executable(goto src/goto.cpp)
add_dependencies(goto ${catkin_EXPORTED_TARGETS})
target_link_libraries(goto
${catkin_LIBRARIES}
)

add_executable(goto_donatello src/goto_donatello.cpp)
add_dependencies(goto_donatello ${catkin_EXPORTED_TARGETS})
target_link_libraries(goto_donatello
${catkin_LIBRARIES}
)

add_executable(goto_michelangelo src/goto_michelangelo.cpp)
add_dependencies(goto_michelangelo ${catkin_EXPORTED_TARGETS})
target_link_libraries(goto_michelangelo
${catkin_LIBRARIES}
)


add_executable(goto_boticelli src/goto_boticelli.cpp)
add_dependencies(goto_boticelli ${catkin_EXPORTED_TARGETS})
target_link_libraries(goto_boticelli
${catkin_LIBRARIES}
)


add_executable(CoM_sample_input_test src/CoM_sample_input_test.cpp)
add_dependencies(CoM_sample_input_test ${catkin_EXPORTED_TARGETS})
target_link_libraries(CoM_sample_input_test
${catkin_LIBRARIES}
)

add_executable(formation_translation src/formation_translation.cpp)
add_dependencies(formation_translation ${catkin_EXPORTED_TARGETS})
target_link_libraries(formation_translation
${catkin_LIBRARIES}
)



add_message_files(FILES Navdata.msg)

catkin_install_python(PROGRAMS
    src/draw_projector.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )



