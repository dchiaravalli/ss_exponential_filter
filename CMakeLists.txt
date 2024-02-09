cmake_minimum_required(VERSION 2.8.3)
project(ss_exponential_filter)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)
find_package(orocos_kdl REQUIRED)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs 
  roscpp rospy std_msgs trajectory_msgs message_generation tf sensor_msgs kdl_parser kdl_conversions geometry_msgs)



find_package( Eigen3 REQUIRED )
include_directories(
  include ${catkin_INCLUDE_DIRS} /usr/include/eigen3 )

add_message_files(
  FILES
  motion_measure.msg
    ss_input_trajectory.msg
)

add_service_files(
 FILES
  force_object_pose.srv
    sloshing_trajectory.srv
    sloshing_point.srv
 )

add_action_files(FILES
  LocateObject.action
)

 generate_messages(
   DEPENDENCIES
   actionlib_msgs
   std_msgs
   trajectory_msgs
   sensor_msgs
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ss_exponential_filter
  CATKIN_DEPENDS
  message_runtime roscpp rospy std_msgs
  DEPENDS orocos_kdl
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
   include/
   include/ss_exponential_filter
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/SS_filter.cpp
   src/${PROJECT_NAME}/FIRFilter.cpp
   src/${PROJECT_NAME}/ComauSSGenerator.cpp
   src/${PROJECT_NAME}/ComauIK.cpp
   src/${PROJECT_NAME}/SS_parameter_evaluator.cpp
   src/${PROJECT_NAME}/Tool_posefinder.cpp
   src/${PROJECT_NAME}/SS_online_control.cpp
   src/${PROJECT_NAME}/SS_offline_control.cpp
   src/${PROJECT_NAME}/Liquid_handler.cpp
   src/${PROJECT_NAME}/SS_line_control.cpp
 )
 target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})
 add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(sloshing_suppression_service src/sloshing_suppression_service.cpp)
target_link_libraries(sloshing_suppression_service ${catkin_LIBRARIES} ss_exponential_filter)
add_dependencies(sloshing_suppression_service ss_exponential_filter_generate_services_cpp)

add_executable(geomagic_trajectory_recorder src/geomagic_trajectory_recorder.cpp)
add_dependencies(geomagic_trajectory_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(geomagic_trajectory_recorder ss_exponential_filter_generate_messages_cpp)
target_link_libraries(geomagic_trajectory_recorder ${catkin_LIBRARIES})

add_executable(sloshing_suppression_online_total src/sloshing_suppression_online_total.cpp)
add_dependencies(sloshing_suppression_online_total ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sloshing_suppression_online_total ${catkin_LIBRARIES} ss_exponential_filter ${orocos_kdl_LIBRARIES})

add_executable(sloshing_suppression_offline_main src/sloshing_suppression_offline_main.cpp)
add_dependencies(sloshing_suppression_offline_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(sloshing_suppression_offline_main ss_exponential_filter_generate_services_cpp ss_exponential_filter_generate_messages_cpp)
target_link_libraries(sloshing_suppression_offline_main ${catkin_LIBRARIES})

add_executable(vicon_trajectory_recorder src/vicon_trajectory_recorder.cpp)
add_dependencies(vicon_trajectory_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(vicon_trajectory_recorder ss_exponential_filter_generate_messages_cpp)
target_link_libraries(vicon_trajectory_recorder ${catkin_LIBRARIES})

add_executable(sloshing_suppression_offline_total src/sloshing_suppression_offline_total.cpp)
add_dependencies(sloshing_suppression_offline_total ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sloshing_suppression_offline_total ${catkin_LIBRARIES} ss_exponential_filter)

add_executable(testnode src/testnode.cpp)
add_dependencies(testnode ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(testnode ${catkin_LIBRARIES} ss_exponential_filter)

add_executable(sloshing_suppression_online_sensor_nonstop src/sloshing_suppression_online_sensor_nonstop.cpp)
add_dependencies(sloshing_suppression_online_sensor_nonstop ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sloshing_suppression_online_sensor_nonstop ${catkin_LIBRARIES} ss_exponential_filter ${orocos_kdl_LIBRARIES})

add_executable(sloshing_suppression_online src/sloshing_suppression_online.cpp)
add_dependencies(sloshing_suppression_online ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sloshing_suppression_online ${catkin_LIBRARIES} ss_exponential_filter)

add_executable(motion_planner_fir src/motion_planner_fir.cpp)
add_dependencies(motion_planner_fir ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(motion_planner_fir ${catkin_LIBRARIES} ss_exponential_filter ${orocos_kdl_LIBRARIES})

add_executable(sloshing_suppression_offline src/sloshing_suppression_offline.cpp)
add_dependencies(sloshing_suppression_offline ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sloshing_suppression_offline ${catkin_LIBRARIES} ss_exponential_filter)

add_executable(force_sensor_recorder src/force_sensor_recorder.cpp)
add_dependencies(force_sensor_recorder ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(force_sensor_recorder ss_exponential_filter_generate_messages_cpp)
target_link_libraries(force_sensor_recorder ${catkin_LIBRARIES} ss_exponential_filter ${orocos_kdl_LIBRARIES})

add_executable(joints_to_ef src/joints_to_ef.cpp)
add_dependencies(joints_to_ef ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(joints_to_ef ss_exponential_filter_generate_messages_cpp)
target_link_libraries(joints_to_ef ${catkin_LIBRARIES} ss_exponential_filter ${orocos_kdl_LIBRARIES})

add_executable(force_sensor_labeller
  src/force_sensor_labeller.cpp
)
add_dependencies(force_sensor_labeller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(force_sensor_labeller
  ${catkin_LIBRARIES} ss_exponential_filter
)

add_executable(object_force_locator
  src/object_force_locator.cpp
)
add_dependencies(object_force_locator ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(object_force_locator
  ${catkin_LIBRARIES} ss_exponential_filter
)

add_executable(vicon_reference_publisher
  src/vicon_reference_publisher.cpp
)
add_dependencies(vicon_reference_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(vicon_reference_publisher
  ${catkin_LIBRARIES} ss_exponential_filter
)


add_executable(sloshing_suppresion_line_main
  src/sloshing_suppresion_line_main.cpp
)
add_dependencies(sloshing_suppresion_line_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sloshing_suppresion_line_main
  ${catkin_LIBRARIES} ss_exponential_filter
)

add_executable(sloshing_suppression_line_continuous
  src/sloshing_suppression_line_continuous.cpp
)
add_dependencies(sloshing_suppression_line_continuous ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sloshing_suppression_line_continuous
  ${catkin_LIBRARIES} ss_exponential_filter
)

add_executable(object_locator_server
  src/object_locator_server.cpp
)
add_dependencies(object_locator_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ss_exponential_filter_generate_actions_cpp)
target_link_libraries(object_locator_server
  ${catkin_LIBRARIES} ss_exponential_filter
)

add_executable(rigid_object_locator_server
  src/rigid_object_locator_server.cpp
)
add_dependencies(rigid_object_locator_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ss_exponential_filter_generate_actions_cpp)
target_link_libraries(rigid_object_locator_server
  ${catkin_LIBRARIES} ss_exponential_filter
)

