cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
project(controleDrone)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  qualisys
  mavros
	gazebo_msgs ##?
)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
  roscpp
  tf
  eigen
  qualisys
  mavros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${eigen3_INCLUDE_DIRS}
)

add_executable(modeldyn src/dynamic_model.cpp)
add_executable(controlesimple src/control.cpp)
add_executable(processinputs src/inputs.cpp src/ekf_imu_mocap_transfert)
add_executable(processoutputs src/outputs.cpp)
add_executable(link_gazebo src/link_gazebo.cpp)
add_executable(trajectory src/trajectory.cpp)
## add_executable(setpoint_publisher src/desiredposepub.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(modeldyn
   ${catkin_LIBRARIES}
 )
target_link_libraries(controlesimple
   ${catkin_LIBRARIES}
 )
target_link_libraries(processinputs
   ${catkin_LIBRARIES}
 )
target_link_libraries(processoutputs
   ${catkin_LIBRARIES}
 )
target_link_libraries(link_gazebo
   ${catkin_LIBRARIES}
 )
target_link_libraries(trajectory
   ${catkin_LIBRARIES}
 )
##target_link_libraries(setpoint_publisher
##   ${catkin_LIBRARIES}
## )

