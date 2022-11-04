# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "siam_main: 3 messages, 0 services")

set(MSG_I_FLAGS "-Isiam_main:/opt/ros/noetic/share/siam_sim/src/siam_main/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(siam_main_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg" NAME_WE)
add_custom_target(_siam_main_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "siam_main" "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg" "geometry_msgs/Quaternion:geometry_msgs/Pose:geometry_msgs/Vector3:geometry_msgs/Twist:geometry_msgs/Point"
)

get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg" NAME_WE)
add_custom_target(_siam_main_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "siam_main" "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg" ""
)

get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg" NAME_WE)
add_custom_target(_siam_main_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "siam_main" "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg" "siam_main/Waypoint"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/siam_main
)
_generate_msg_cpp(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/siam_main
)
_generate_msg_cpp(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/siam_main
)

### Generating Services

### Generating Module File
_generate_module_cpp(siam_main
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/siam_main
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(siam_main_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(siam_main_generate_messages siam_main_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_cpp _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_cpp _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_cpp _siam_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(siam_main_gencpp)
add_dependencies(siam_main_gencpp siam_main_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS siam_main_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/siam_main
)
_generate_msg_eus(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/siam_main
)
_generate_msg_eus(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/siam_main
)

### Generating Services

### Generating Module File
_generate_module_eus(siam_main
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/siam_main
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(siam_main_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(siam_main_generate_messages siam_main_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_eus _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_eus _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_eus _siam_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(siam_main_geneus)
add_dependencies(siam_main_geneus siam_main_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS siam_main_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/siam_main
)
_generate_msg_lisp(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/siam_main
)
_generate_msg_lisp(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/siam_main
)

### Generating Services

### Generating Module File
_generate_module_lisp(siam_main
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/siam_main
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(siam_main_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(siam_main_generate_messages siam_main_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_lisp _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_lisp _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_lisp _siam_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(siam_main_genlisp)
add_dependencies(siam_main_genlisp siam_main_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS siam_main_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/siam_main
)
_generate_msg_nodejs(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/siam_main
)
_generate_msg_nodejs(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/siam_main
)

### Generating Services

### Generating Module File
_generate_module_nodejs(siam_main
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/siam_main
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(siam_main_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(siam_main_generate_messages siam_main_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_nodejs _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_nodejs _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_nodejs _siam_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(siam_main_gennodejs)
add_dependencies(siam_main_gennodejs siam_main_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS siam_main_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Twist.msg;/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/siam_main
)
_generate_msg_py(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/siam_main
)
_generate_msg_py(siam_main
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/siam_main
)

### Generating Services

### Generating Module File
_generate_module_py(siam_main
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/siam_main
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(siam_main_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(siam_main_generate_messages siam_main_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Telemetry.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_py _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Waypoint.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_py _siam_main_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/opt/ros/noetic/share/siam_sim/src/siam_main/msg/Uplan.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_py _siam_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(siam_main_genpy)
add_dependencies(siam_main_genpy siam_main_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS siam_main_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/siam_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/siam_main
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(siam_main_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(siam_main_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/siam_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/siam_main
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(siam_main_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(siam_main_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/siam_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/siam_main
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(siam_main_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(siam_main_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/siam_main)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/siam_main
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(siam_main_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(siam_main_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/siam_main)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/siam_main\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/siam_main
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(siam_main_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(siam_main_generate_messages_py geometry_msgs_generate_messages_py)
endif()
