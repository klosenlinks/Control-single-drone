cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 11)
project(control_drone)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  qualisys
  mavros
  gazebo_msgs 
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
add_executable(controlePID src/controlPID.cpp)
add_executable(controleSM src/controlSM.cpp)
add_executable(processinputs src/inputs.cpp src/ekf_imu_mocap_transfert.cpp)
add_executable(processoutputs src/outputs.cpp)
add_executable(link_gazebo src/link_gazebo.cpp)
add_executable(link_gazebo_perturbation src/link_gazebo_perturbation.cpp)
add_executable(trajectory src/trajectory.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(modeldyn
   ${catkin_LIBRARIES}
 )
target_link_libraries(controlePID
   ${catkin_LIBRARIES}
 )
target_link_libraries(controleSM
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
target_link_libraries(link_gazebo_perturbation
   ${catkin_LIBRARIES}
 )
target_link_libraries(trajectory
   ${catkin_LIBRARIES}
 )
