cmake_minimum_required(VERSION 2.8.3)
project(xr1controllerol)



add_compile_options(-std=c++11)
add_compile_options(-fPIC)
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  xr1controllerros
  roslib
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES xr1controllerol
  CATKIN_DEPENDS roscpp rospy std_msgs
)


include_directories(
	include
	include/${PROJECT_NAME}
	xr1controller-build
	xr1controllerpm-build
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
)


add_executable(actuator_bridge src/actuator_bridge.cpp) 
target_link_libraries(actuator_bridge ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(actuator_bridge ${PROJECT_NAME})
