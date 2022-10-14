# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "siam_main: 1 messages, 0 services")

set(MSG_I_FLAGS "-Isiam_main:C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/msg;-Istd_msgs:C:/Program Files/MATLAB/R2022a/sys/ros1/win64/ros1/share/std_msgs/cmake/../msg;-Igeometry_msgs:C:/Program Files/MATLAB/R2022a/sys/ros1/win64/ros1/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(siam_main_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/msg/FlightPlan.msg" NAME_WE)
add_custom_target(_siam_main_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "siam_main" "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/msg/FlightPlan.msg" "geometry_msgs/Point"
)

#
#  langs = gencpp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(siam_main
  "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/msg/FlightPlan.msg"
  "${MSG_I_FLAGS}"
  "C:/Program Files/MATLAB/R2022a/sys/ros1/win64/ros1/share/geometry_msgs/cmake/../msg/Point.msg"
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
get_filename_component(_filename "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/msg/FlightPlan.msg" NAME_WE)
add_dependencies(siam_main_generate_messages_cpp _siam_main_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(siam_main_gencpp)
add_dependencies(siam_main_gencpp siam_main_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS siam_main_generate_messages_cpp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(siam_main
  "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/msg/FlightPlan.msg"
  "${MSG_I_FLAGS}"
  "C:/Program Files/MATLAB/R2022a/sys/ros1/win64/ros1/share/geometry_msgs/cmake/../msg/Point.msg"
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
get_filename_component(_filename "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/msg/FlightPlan.msg" NAME_WE)
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

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/siam_main)
  install(CODE "execute_process(COMMAND \"C:/Users/usuario/AppData/Roaming/MathWorks/MATLAB/R2022a/ros1/win64/venv/Scripts/python.exe\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/siam_main\")")
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
