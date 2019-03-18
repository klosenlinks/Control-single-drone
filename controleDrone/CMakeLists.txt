cmake_minimum_required(VERSION 3.1)##2.8.3
set(CMAKE_CXX_STANDARD 11)
project(controleDrone)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  qualisys
  mavros
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
## add_executable(setpoint_publisher src/desiredposepub.cpp)
add_executable(processinputs src/inputs.cpp src/ekf_imu_mocap_transfert)
add_executable(processoutputs src/outputs.cpp)

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
##target_link_libraries(setpoint_publisher
##   ${catkin_LIBRARIES}
## )

