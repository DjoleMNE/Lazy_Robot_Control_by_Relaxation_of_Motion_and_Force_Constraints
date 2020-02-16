cmake_minimum_required(VERSION 3.5.1)
project(Controller)
#set(BUILD_SHARED_LIBS OFF)

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

set(ROBOT youBot CACHE STRING "Robot ID")
message("---- Selected Robot: ${ROBOT}")

if("${ROBOT}" STREQUAL "kinova")
  if(NOT KORTEX_SUB_DIR)
    set(KORTEX_SUB_DIR "cpp/")
  else()
    set(KORTEX_SUB_DIR "${KORTEX_SUB_DIR}/")
  endif()

  #use -DKORTEX_SUB_DIR=cpp
  set(KORTEX_DIR "${PROJECT_SOURCE_DIR}/../kortex_api/${KORTEX_SUB_DIR}")
  message("---- KORTEX_DIR: " ${KORTEX_DIR})

  # Activate C++ 11
  set (CMAKE_CXX_STANDARD 11)
  include_directories(${PROJECT_SOURCE_DIR}/thirdParty/cxxopts/)

  add_compile_options(-Wall)
  add_compile_options(-Wno-reorder)

  add_definitions(-D_OS_UNIX)
  ##### Link to lib
  link_libraries(${KORTEX_DIR}lib/${CMAKE_BUILD_TYPE}/libKortexApi.a)

  # link other libs
  link_libraries(pthread)

  # add include dir
  include_directories(${KORTEX_DIR}include)
  include_directories(${KORTEX_DIR}include/client)
  include_directories(${KORTEX_DIR}include/common)
  include_directories(${KORTEX_DIR}include/messages)
  include_directories(${KORTEX_DIR}include/client_stubs)

  # Link to submodule base on OS
  link_libraries(pthread)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(orocos_kdl REQUIRED orocos_kdl)

#find_package(orocos_kdl PATHS /home/djole/Master/Thesis/GIT/MT_testing/KDL/KDL_install_dir/)
find_package(kdl_parser)
find_package(Boost COMPONENTS system filesystem REQUIRED)
#find_package(youbot_driver)

include_directories(
  include
  ${orocos_kdl_INCLUDE_DIRS}
  ${kdl_parser_INCLUDE_DIRS}
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

if("${ROBOT}" STREQUAL "lwr")
  add_executable(main
    src/main_lwr.cpp
    src/constants.cpp
    src/kdl_eigen_conversions.cpp
    src/moving_variance.cpp
    src/moving_slope.cpp
    src/geometry_utils.cpp
    src/model_prediction.cpp
    src/finite_state_machine.cpp
    src/motion_profile.cpp
    src/solver_vereshchagin.cpp
    src/solver_recursive_newton_euler.cpp
    src/fk_vereshchagin.cpp
    src/lwr_mediator.cpp
    src/lwr_kdl_model.cpp
    src/safety_controller.cpp
    src/abag.cpp
    src/dynamics_controller.cpp
  )

elseif("${ROBOT}" STREQUAL "kinova")
  add_executable(main
    src/main_kinova.cpp
    src/constants.cpp
    src/kdl_eigen_conversions.cpp
    src/moving_variance.cpp
    src/moving_slope.cpp
    src/geometry_utils.cpp
    src/model_prediction.cpp
    src/finite_state_machine.cpp
    src/motion_profile.cpp
    src/solver_vereshchagin.cpp
    src/solver_recursive_newton_euler.cpp
    src/fk_vereshchagin.cpp
    src/lwr_mediator.cpp
    src/lwr_kdl_model.cpp
    src/safety_controller.cpp
    src/abag.cpp
    src/dynamics_controller.cpp
  )

else()
  add_executable(main
    src/main_youbot.cpp
    src/constants.cpp
    src/kdl_eigen_conversions.cpp
    src/geometry_utils.cpp
    src/moving_variance.cpp
    src/moving_slope.cpp
    src/model_prediction.cpp
    src/finite_state_machine.cpp
    src/motion_profile.cpp
    src/solver_vereshchagin.cpp
    src/solver_recursive_newton_euler.cpp
    src/fk_vereshchagin.cpp
    src/youbot_mediator.cpp
    src/youbot_custom_model.cpp
    src/safety_controller.cpp
    src/abag.cpp
    src/dynamics_controller.cpp
  )
endif()

target_link_libraries(main
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    YouBotDriver
    soem
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
)

if("${ROBOT}" STREQUAL "youBot")
  ## run sudo command to enable direct network access
  option(${PROJECT_NAME}_USE_SETCAP "Set permissions to access ethernet interface without sudo" ON)

  set(SUDO_COMMAND sudo)

  if(${PROJECT_NAME}_USE_SETCAP)
      add_custom_command(TARGET main POST_BUILD
      COMMAND ${SUDO_COMMAND} setcap cap_net_raw+ep $<TARGET_FILE:main>
  )    
  endif()
endif()