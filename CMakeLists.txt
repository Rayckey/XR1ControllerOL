cmake_minimum_required(VERSION 2.8.3)
project(xr1controllerol)



add_compile_options(-std=c++11)
add_compile_options(-fPIC)
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)
add_compile_options(-DIMU_ENABLE)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  xr1controllerros
  roslib
  tf_conversions
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xr1controllerol
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs
)


include_directories(
	include
	include/${PROJECT_NAME}
	xr1controller-build
	xr1controllerpm-build
  xr1imu-build
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/serialPort_Ethernet2CAN_sdk_v3.0.0/sdk/include
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/serialPort_Ethernet2CAN_sdk_v3.0.0/sdk/include/asio
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/serialPort_Ethernet2CAN_sdk_v3.0.0/sdk/include/asio/asio
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/Eigen3
)




# Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/xr1controllerol.cpp
 )



## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/serialPort_Ethernet2CAN_sdk_v3.0.0/sdk/lib/linux_x86_64/libactuatorController.so
   ${CMAKE_CURRENT_SOURCE_DIR}/xr1controllerpm-build/libXR1ControllerPM.so
   ${CMAKE_CURRENT_SOURCE_DIR}/xr1controller-build/libXR1Controller.so
   ${CMAKE_CURRENT_SOURCE_DIR}/xr1imu-build/libXR1IMU.so
)


add_executable(actuator_bridge src/actuator_bridge.cpp) 
target_link_libraries(actuator_bridge ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(actuator_bridge ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(tilt_test src/tilt_test.cpp) 
target_link_libraries(tilt_test ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(tilt_test ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(xr1controllerIK_simulator src/xr1controllerIK_simulator.cpp) 
target_link_libraries(xr1controllerIK_simulator ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(xr1controllerIK_simulator ${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(xr1controllerodom_simulator src/xr1controllerodom_simulator.cpp) 
target_link_libraries(xr1controllerodom_simulator ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(xr1controllerodom_simulator ${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(mute_test src/mute_test.cpp) 
target_link_libraries(mute_test ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(mute_test ${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(teach_mode src/teach_mode.cpp) 
target_link_libraries(teach_mode ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(teach_mode ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(taught_test src/taught_test.cpp) 
target_link_libraries(taught_test ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(taught_test ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(xr1controllerIK_varify src/xr1controllerIK_varify.cpp) 
target_link_libraries(xr1controllerIK_varify ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(xr1controllerIK_varify ${PROJECT_NAME} ${catkin_LIBRARIES})
