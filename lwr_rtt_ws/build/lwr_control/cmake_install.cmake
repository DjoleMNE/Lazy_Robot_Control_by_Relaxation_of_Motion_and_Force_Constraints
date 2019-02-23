# Install script for directory: /home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/src/lwr_control

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install")
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

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
        file(MAKE_DIRECTORY "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}")
      endif()
      if (NOT EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin")
        file(WRITE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/.catkin" "")
      endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/_setup_util.py")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install" TYPE PROGRAM FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/catkin_generated/installspace/_setup_util.py")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/env.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install" TYPE PROGRAM FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/catkin_generated/installspace/env.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/setup.bash")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install" TYPE FILE FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/catkin_generated/installspace/setup.bash")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/setup.sh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install" TYPE FILE FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/catkin_generated/installspace/setup.sh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/setup.zsh")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install" TYPE FILE FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/catkin_generated/installspace/setup.zsh")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/.rosinstall")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install" TYPE FILE FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/catkin_generated/installspace/.rosinstall")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/lwr_control/liblwr_control-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/lwr_control/liblwr_control-gnulinux.so")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/lwr_control/liblwr_control-gnulinux.so"
         RPATH "/home/djole/Master/LWR/software/isir/rtt_ros-2.9_ws/install/lib:/home/djole/Master/LWR/software/isir/rtt_ros-2.9_ws/install/lib/orocos/gnulinux/rtt_ros/plugins:/home/djole/Master/LWR/software/isir/rtt_ros-2.9_ws/install/lib/orocos/gnulinux/rtt_ros/types:/home/djole/Master/LWR/software/isir/orocos-2.9_ws/install/lib:/opt/ros/kinetic/lib:/home/djole/Master/LWR/software/isir/lwr_ws/install/lib:/home/djole/Master/LWR/software/isir/orocos-2.9_ws/install/lib/orocos/gnulinux:/home/djole/Master/LWR/software/isir/orocos-2.9_ws/install/lib/orocos/gnulinux/plugins:/home/djole/Master/LWR/software/isir/orocos-2.9_ws/install/lib/orocos/gnulinux/types:/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/lib/orocos/gnulinux/lwr_control:/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/lib/orocos/gnulinux/lwr_control/types:/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/lib/orocos/gnulinux/lwr_control/plugins:/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/lib")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/lwr_control" TYPE SHARED_LIBRARY FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/devel/.private/lwr_control/lib/orocos/gnulinux/lwr_control/liblwr_control-gnulinux.so")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/lwr_control/liblwr_control-gnulinux.so" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/lwr_control/liblwr_control-gnulinux.so")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/lwr_control/liblwr_control-gnulinux.so"
         OLD_RPATH "/home/djole/Master/LWR/software/isir/orocos-2.9_ws/install/lib:/home/djole/Master/LWR/software/isir/rtt_ros-2.9_ws/install/lib:/home/djole/Master/LWR/software/isir/rtt_ros-2.9_ws/install/lib/orocos/gnulinux/rtt_ros/plugins:/home/djole/Master/LWR/software/isir/rtt_ros-2.9_ws/install/lib/orocos/gnulinux/rtt_ros/types:/opt/ros/kinetic/lib:/home/djole/Master/LWR/software/isir/lwr_ws/install/lib:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::"
         NEW_RPATH "/home/djole/Master/LWR/software/isir/rtt_ros-2.9_ws/install/lib:/home/djole/Master/LWR/software/isir/rtt_ros-2.9_ws/install/lib/orocos/gnulinux/rtt_ros/plugins:/home/djole/Master/LWR/software/isir/rtt_ros-2.9_ws/install/lib/orocos/gnulinux/rtt_ros/types:/home/djole/Master/LWR/software/isir/orocos-2.9_ws/install/lib:/opt/ros/kinetic/lib:/home/djole/Master/LWR/software/isir/lwr_ws/install/lib:/home/djole/Master/LWR/software/isir/orocos-2.9_ws/install/lib/orocos/gnulinux:/home/djole/Master/LWR/software/isir/orocos-2.9_ws/install/lib/orocos/gnulinux/plugins:/home/djole/Master/LWR/software/isir/orocos-2.9_ws/install/lib/orocos/gnulinux/types:/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/lib/orocos/gnulinux/lwr_control:/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/lib/orocos/gnulinux/lwr_control/types:/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/lib/orocos/gnulinux/lwr_control/plugins:/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/orocos/gnulinux/lwr_control/liblwr_control-gnulinux.so")
    endif()
  endif()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/orocos/lwr_control" TYPE FILE FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/src/lwr_control/include/lwr_control/lwr_control.hpp")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/lwr_control-gnulinux.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(MAKE_DIRECTORY $ENV{DESTDIR}/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/install/lib/orocos/gnulinux/lwr_control)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/catkin_generated/installspace/lwr_control.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_control/cmake" TYPE FILE FILES
    "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/catkin_generated/installspace/lwr_controlConfig.cmake"
    "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/catkin_generated/installspace/lwr_controlConfig-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_control" TYPE FILE FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/src/lwr_control/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_control/scripts" TYPE DIRECTORY FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/src/lwr_control/scripts/" REGEX "/\\.svn$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/lwr_control/launch" TYPE DIRECTORY FILES "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/src/lwr_control/launch/" REGEX "/\\.svn$" EXCLUDE)
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/gtest/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/djole/Master/Thesis/GIT/MT_testing/lwr_rtt_ws/build/lwr_control/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
