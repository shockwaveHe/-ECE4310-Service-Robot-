cmake_minimum_required(VERSION 2.8.3)

project(spark1_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  move_base_msgs
  sensor_msgs
  tf
  actionlib_msgs
  nav_msgs
  ar_track_alvar_msgs
)

catkin_package(

  CATKIN_DEPENDS 
        roscpp 
        std_msgs 
        geometry_msgs 
        move_base_msgs
        sensor_msgs
        tf 
        actionlib_msgs 
        nav_msgs 
        ar_track_alvar_msgs

)
include_directories(include ${catkin_INCLUDE_DIRS} ) 

add_executable(move_publisher src/move_pub.cpp)
target_link_libraries(move_publisher ${catkin_LIBRARIES})
add_dependencies(move_publisher ${PROJECT_NAME}_generate_messages_cpp)

add_executable(parameter_config src/parameter_config.cpp)
target_link_libraries(parameter_config ${catkin_LIBRARIES})
add_dependencies(parameter_config ${PROJECT_NAME}_generate_messages_cpp)

add_executable(service_robot src/service_robot.cpp)
target_link_libraries(service_robot ${catkin_LIBRARIES})

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
