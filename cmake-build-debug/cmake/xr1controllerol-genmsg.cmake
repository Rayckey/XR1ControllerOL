# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "xr1controllerol: 1 messages, 9 services")

set(MSG_I_FLAGS "-Ixr1controllerol:/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(xr1controllerol_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv" ""
)

get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv" ""
)

get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv" "std_msgs/MultiArrayDimension:std_msgs/Float64MultiArray:std_msgs/MultiArrayLayout"
)

get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv" "geometry_msgs/Quaternion:geometry_msgs/Transform:geometry_msgs/Vector3"
)

get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv" ""
)

get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg" ""
)

get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv" ""
)

get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv" "geometry_msgs/Quaternion:geometry_msgs/Transform:geometry_msgs/Vector3"
)

get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv" ""
)

get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv" NAME_WE)
add_custom_target(_xr1controllerol_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "xr1controllerol" "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)

### Generating Services
_generate_srv_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_cpp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
)

### Generating Module File
_generate_module_cpp(xr1controllerol
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(xr1controllerol_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(xr1controllerol_generate_messages xr1controllerol_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_cpp _xr1controllerol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xr1controllerol_gencpp)
add_dependencies(xr1controllerol_gencpp xr1controllerol_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xr1controllerol_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)

### Generating Services
_generate_srv_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)
_generate_srv_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)
_generate_srv_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)
_generate_srv_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)
_generate_srv_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)
_generate_srv_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)
_generate_srv_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)
_generate_srv_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)
_generate_srv_eus(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
)

### Generating Module File
_generate_module_eus(xr1controllerol
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(xr1controllerol_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(xr1controllerol_generate_messages xr1controllerol_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_eus _xr1controllerol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xr1controllerol_geneus)
add_dependencies(xr1controllerol_geneus xr1controllerol_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xr1controllerol_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)

### Generating Services
_generate_srv_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)
_generate_srv_lisp(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
)

### Generating Module File
_generate_module_lisp(xr1controllerol
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(xr1controllerol_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(xr1controllerol_generate_messages xr1controllerol_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_lisp _xr1controllerol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xr1controllerol_genlisp)
add_dependencies(xr1controllerol_genlisp xr1controllerol_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xr1controllerol_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)

### Generating Services
_generate_srv_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)
_generate_srv_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)
_generate_srv_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)
_generate_srv_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)
_generate_srv_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)
_generate_srv_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)
_generate_srv_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)
_generate_srv_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)
_generate_srv_nodejs(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
)

### Generating Module File
_generate_module_nodejs(xr1controllerol
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(xr1controllerol_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(xr1controllerol_generate_messages xr1controllerol_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_nodejs _xr1controllerol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xr1controllerol_gennodejs)
add_dependencies(xr1controllerol_gennodejs xr1controllerol_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xr1controllerol_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)

### Generating Services
_generate_srv_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)
_generate_srv_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)
_generate_srv_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayDimension.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Float64MultiArray.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/MultiArrayLayout.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)
_generate_srv_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)
_generate_srv_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)
_generate_srv_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)
_generate_srv_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Transform.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Vector3.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)
_generate_srv_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)
_generate_srv_py(xr1controllerol
  "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
)

### Generating Module File
_generate_module_py(xr1controllerol
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(xr1controllerol_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(xr1controllerol_generate_messages xr1controllerol_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKTrackingService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/RobotStateQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationOverwrite.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKPlannerService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/HandGripQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/msg/AnimationMsgs.msg" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/BalanceQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/IKLinearService.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/askReadiness.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/media/fcz/work/work/SRC/XR1/xr1_catkin_ws/src/xr1controllerol/srv/AnimationQuery.srv" NAME_WE)
add_dependencies(xr1controllerol_generate_messages_py _xr1controllerol_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(xr1controllerol_genpy)
add_dependencies(xr1controllerol_genpy xr1controllerol_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS xr1controllerol_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/xr1controllerol
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(xr1controllerol_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(xr1controllerol_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/xr1controllerol
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(xr1controllerol_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(xr1controllerol_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/xr1controllerol
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(xr1controllerol_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(xr1controllerol_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/xr1controllerol
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(xr1controllerol_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(xr1controllerol_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/xr1controllerol
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(xr1controllerol_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(xr1controllerol_generate_messages_py geometry_msgs_generate_messages_py)
endif()
