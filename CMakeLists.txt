cmake_minimum_required(VERSION 2.8.3)
project(Tuesday)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)

################################################################################
# Declare ROS messages, services and actions
################################################################################
## Generate messages in the 'msg' folder
add_message_files(
  FILES
  Open.msg
)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES Tuesday
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
  /usr/local/include/sphinxbase
  /usr/local/include/pocketsphinx
  )


# target_link_libraries(
#   ${catkin_LIBRARIES}
#   )

# target_link_libraries(
#   ${catkin_LIBRARIES})
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
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

catkin_install_python(
  PROGRAMS
  scripts/servo.py
  scripts/move_arm.py
  scripts/detect.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_Tuesday.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
