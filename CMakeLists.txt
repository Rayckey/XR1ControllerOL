cmake_minimum_required(VERSION 2.8.3)
project(xr1controllerol)



add_compile_options(-std=c++11)
add_compile_options(-fPIC)
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)
add_compile_options(-DIMU_ENABLE)
add_compile_options(-O3)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  xr1controllerros
		std_msgs
		std_srvs
		geometry_msgs
		message_generation
  		roslib
  		tf_conversions
		eigen_conversions
)



include_directories(
	include
	include/${PROJECT_NAME}
	xr1controllers-build
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/serialPort_Ethernet2CAN_sdk_v3.0.0/sdk/include
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/serialPort_Ethernet2CAN_sdk_v3.0.0/sdk/include/asio
	${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/serialPort_Ethernet2CAN_sdk_v3.0.0/sdk/include/asio/asio
  	${catkin_INCLUDE_DIRS}
  	${CMAKE_CURRENT_SOURCE_DIR}/Eigen3
)


 add_message_files(
   FILES
		 AnimationMsgs.msg
 )

add_service_files(
		FILES
		IKLinearService.srv
		IKTrackingService.srv
		HandGripQuery.srv
		askReadiness.srv
		AnimationQuery.srv
		RobotStateQuery.srv
)

generate_messages(
		DEPENDENCIES
		std_msgs
		geometry_msgs
)

catkin_package(
		INCLUDE_DIRS include
		LIBRARIES xr1controllerol
		CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs
)



# Declare a C++ library
 add_library(${PROJECT_NAME}
   src/${PROJECT_NAME}/xr1controllerol.cpp
		 src/${PROJECT_NAME}/xr1controllerolulit.cpp
		 src/${PROJECT_NAME}/xr1controllerolmsgulit.cpp
		 src/${PROJECT_NAME}/xr1controllerolanimation.cpp
		 src/${PROJECT_NAME}/xr1controllerolhigh.cpp
		 src/${PROJECT_NAME}/xr1controllerollow.cpp
		 src/${PROJECT_NAME}/xr1controllerolik.cpp
		 src/${PROJECT_NAME}/xr1controllerolmodes.cpp
		 src/${PROJECT_NAME}/xr1controllerolactuator.cpp
		 src/${PROJECT_NAME}/xr1controllerolomni.cpp
		 src/${PROJECT_NAME}/xr1controllerolhopps.cpp
		 src/${PROJECT_NAME}/xr1controlleroltilt.cpp
 )



## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${CMAKE_CURRENT_SOURCE_DIR}/ActuatorController_SDK/serialPort_Ethernet2CAN_sdk_v3.0.0/sdk/lib/$ENV{BUILD_PLATFORM}/libActuatorController.so
   ${CMAKE_CURRENT_SOURCE_DIR}/xr1controllers-build/$ENV{BUILD_PLATFORM}/libXR1ControllerPM.so
   ${CMAKE_CURRENT_SOURCE_DIR}/xr1controllers-build/$ENV{BUILD_PLATFORM}/libXR1Controller.so
   ${CMAKE_CURRENT_SOURCE_DIR}/xr1controllers-build/$ENV{BUILD_PLATFORM}/libXR1IMU.so
   ${CMAKE_CURRENT_SOURCE_DIR}/xr1controllers-build/$ENV{BUILD_PLATFORM}/libXR1ControllerALP.so
		${CMAKE_CURRENT_SOURCE_DIR}/xr1controllers-build/$ENV{BUILD_PLATFORM}/libXR1ControllerBLC.so
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

add_executable(actuator_bridge src/actuator_bridge.cpp) 
target_link_libraries(actuator_bridge ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(actuator_bridge ${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(tilt_test src/tilt_test.cpp)
#target_link_libraries(tilt_test ${catkin_LIBRARIES} ${PROJECT_NAME})
#add_dependencies(tilt_test ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(xr1controllerIK_simulator src/xr1controllerIK_simulator.cpp)
target_link_libraries(xr1controllerIK_simulator ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(xr1controllerIK_simulator ${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(xr1controllerOL_simulator src/xr1controllerOL_simulator.cpp)
target_link_libraries(xr1controllerOL_simulator ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(xr1controllerOL_simulator ${PROJECT_NAME} ${catkin_LIBRARIES})


#add_executable(xr1controllerodom_simulator src/xr1controllerodom_simulator.cpp)
#target_link_libraries(xr1controllerodom_simulator ${catkin_LIBRARIES} ${PROJECT_NAME})
#add_dependencies(xr1controllerodom_simulator ${PROJECT_NAME} ${catkin_LIBRARIES})


#add_executable(mute_test src/mute_test.cpp)
#target_link_libraries(mute_test ${catkin_LIBRARIES} ${PROJECT_NAME})
#add_dependencies(mute_test ${PROJECT_NAME} ${catkin_LIBRARIES})


#add_executable(taught_test src/taught_test.cpp)
#target_link_libraries(taught_test ${catkin_LIBRARIES} ${PROJECT_NAME})
#add_dependencies(taught_test ${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(xr1controllerIK_varify src/xr1controllerIK_varify.cpp)
#target_link_libraries(xr1controllerIK_varify ${catkin_LIBRARIES} ${PROJECT_NAME})
#add_dependencies(xr1controllerIK_varify ${PROJECT_NAME} ${catkin_LIBRARIES})


add_executable(xr1controllerALP_simulator src/xr1controllerALP_simulator.cpp)
target_link_libraries(xr1controllerALP_simulator ${catkin_LIBRARIES} ${PROJECT_NAME})
add_dependencies(xr1controllerALP_simulator ${PROJECT_NAME} ${catkin_LIBRARIES})




#add_executable(ik_example src/IK_Example.cpp)
#target_link_libraries(ik_example ${catkin_LIBRARIES} ${PROJECT_NAME})
#add_dependencies(ik_example ${catkin_LIBRARIES} ${PROJECT_NAME})


#add_executable(ik_bench src/IK_Bench.cpp)
#target_link_libraries(ik_bench ${catkin_LIBRARIES} ${PROJECT_NAME})
#add_dependencies(ik_bench ${catkin_LIBRARIES} ${PROJECT_NAME})

#add_executable(ik_odom src/IK_odom.cpp)
#target_link_libraries(ik_odom ${catkin_LIBRARIES} ${PROJECT_NAME})
#add_dependencies(ik_odom ${catkin_LIBRARIES} ${PROJECT_NAME})
