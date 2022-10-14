# Install script for directory: C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/siam_main/msg" TYPE FILE FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/msg/FlightPlan.msg")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/siam_main/cmake" TYPE FILE FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/build/siam_main/catkin_generated/installspace/siam_main-msg-paths.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/devel/include/siam_main")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(COMMAND "C:/Users/usuario/AppData/Roaming/MathWorks/MATLAB/R2022a/ros1/win64/venv/Scripts/python.exe" -m compileall "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/devel/lib/site-packages/siam_main")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/site-packages" TYPE DIRECTORY FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/devel/lib/site-packages/siam_main")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/build/siam_main/catkin_generated/installspace/siam_main.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/siam_main/cmake" TYPE FILE FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/build/siam_main/catkin_generated/installspace/siam_main-msg-extras.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/siam_main/cmake" TYPE FILE FILES
    "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/build/siam_main/catkin_generated/installspace/siam_mainConfig.cmake"
    "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/build/siam_main/catkin_generated/installspace/siam_mainConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/siam_main" TYPE FILE FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/include/")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY OPTIONAL FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/devel/lib/siam_main_matlab.lib")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/bin" TYPE SHARED_LIBRARY FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/devel/bin/siam_main_matlab.dll")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/m/" TYPE DIRECTORY FILES "C:/Users/usuario/Documents/i3a_repos/siam_sim/src/matlab_msg_gen_ros1/win64/src/siam_main/m/" FILES_MATCHING REGEX "/[^/]*\\.m$")
endif()

