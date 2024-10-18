# cmake_minimum_required(VERSION 2.8.3)

# project(softactuator)

# find_package(catkin REQUIRED)

# catkin_package()

# find_package(roslaunch)

# foreach(dir config launch meshes urdf)
# 	install(DIRECTORY ${dir}/
# 		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
# endforeach(dir)

#-----------------------------------------------------------------

cmake_minimum_required(VERSION 3.0.2)
project(softactuator)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  gazebo_ros
)

find_package(gazebo REQUIRED)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  SetStiffness.srv
  SetDamping.srv
  SetFriction.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

## Declare catkin package
catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
)

# Add the Gazebo and ROS dependencies
include_directories(
	include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)

# Add the plugin source file from its actual location
add_library(${PROJECT_NAME} SHARED
 ~/catkin_ws/src/JointStiffnessPlugin.cpp
)

# Link the plugin library against Gazebo libraries
target_link_libraries(${PROJECT_NAME}
 ${GAZEBO_LIBRARIES}
 ${catkin_LIBRARIES}
)

# Install the plugin library
#install(TARGETS ${PROJECT_NAME}
# DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

#------------------------------------------------------------------------------------
#add_executable(stiffness_client ~/catkin_ws/src/stiffness_client.cpp)
#add_dependencies(stiffness_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(stiffness_client
#  ${catkin_LIBRARIES}
#)


#install(TARGETS stiffness_client
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#add_executable(damping_client ~/catkin_ws/src/damping_client.cpp)
#add_dependencies(damping_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(damping_client
#  ${catkin_LIBRARIES}
#)


#install(TARGETS damping_client
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

#add_executable(friction_client ~/catkin_ws/src/friction_client.cpp)
#add_dependencies(friction_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(friction_client
#  ${catkin_LIBRARIES}
#)


#install(TARGETS friction_client
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)
