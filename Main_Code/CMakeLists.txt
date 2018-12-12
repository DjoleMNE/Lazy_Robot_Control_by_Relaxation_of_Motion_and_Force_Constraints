cmake_minimum_required(VERSION 2.8.3)
project(Main_Code)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 support. Please use a different C++ compiler.")
endif()


find_package(orocos_kdl)
find_package(kdl_parser)
find_package(Boost COMPONENTS system filesystem REQUIRED)
#find_package(youbot_driver)

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  #${youbot_driver_INCLUDE_DIRS}
  /home/djole/youBot_driver_inst_dir/usr/local/include
  /home/djole/youBot_driver_inst_dir/usr/local/include/youbot_driver/soem
  /home/djole/youBot_driver_inst_dir/usr/local/include/youbot_driver/soem/osal
  /home/djole/youBot_driver_inst_dir/usr/local/include/youbot_driver/soem/oshw/linux
)

link_directories(
  #${youbot_driver_INCLUDE_DIRS}
  /home/djole/youBot_driver_inst_dir/usr/local/lib
  "/home/djole/Master/Thesis/GIT/MT_testing/KDL/KDL_install_dir/lib/"
)


add_executable(main
  src/dynamics_controller.cpp
  src/solver_vereshchagin.cpp
)

target_link_libraries(main
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    YouBotDriver
    soem
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

## run sudo command to enable direct network access
## disabled here by default because it is set by Debian postinst command

option(${PROJECT_NAME}_USE_SETCAP "Set permissions to access ethernet interface without sudo" ON)

set(SUDO_COMMAND sudo)

if(${PROJECT_NAME}_USE_SETCAP)
    add_custom_command(TARGET main POST_BUILD
    COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep $<TARGET_FILE:main>
)    
endif()