################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(my_id_robot)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED
  OpenCV REQUIRED
  COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  )

message("OpenCV version: ${OpenCV_VERSION}")

################################################################################
# Declare ROS messages, services and actions
################################################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Open.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  FindObjectOpenCV.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
#  message_runtime
#  FindObjectOpenCV
#  DEPENDS system_lib
)

################################################################################
# Build
################################################################################
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/sphinxbase
  /usr/local/include/pocketsphinx
  /opt/opencv-4.1.0/include/opencv4
  )

link_directories(${OpenCV_LIB_DIR})

add_executable(find_object_opencv src/find_object_opencv.cpp)
target_link_libraries(
  find_object_opencv
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}  # opt/opencv-4.1.0/lib
  )

add_executable(main_opencv_object src/main_opencv_object.cpp)
target_link_libraries(main_opencv_object
  ${catkin_LIBRARIES})
#add_dependencies(find_object_opencv 
#  ${{PROJECT_NAME}_EXPORTED_TARGETS}
#  ${catkin_EXPORTED_TARGETS})

add_executable(main_id_subscriber src/main_id_subscriber.cpp)
target_link_libraries(main_id_subscriber ${catkin_LIBRARIES})
#add_dependencies(main_subscriber my_robot_generate_messages_cpp)

add_executable(id_continuous src/id_continuous.cpp)
target_link_libraries(id_continuous
  ${catkin_LIBRARIES}
  pocketsphinx
  sphinxbase
  sphinxad
  )
#add_dependencies(continuous my_robot_generate_messages_cpp)

install(
  TARGETS
  main_id_subscriber
  find_object_opencv
  main_opencv_object
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(
  PROGRAMS
  scripts/servo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
