cmake_minimum_required(VERSION 2.8.3)
project(lwr_rtt_control)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#if(NOT CMAKE_BUILD_TYPE)
#    set(CMAKE_BUILD_TYPE Release)
#endif()

find_package(catkin REQUIRED COMPONENTS
    rtt_ros
    cmake_modules
    roscpp
    tf
    tf_conversions
    tf2_ros
    tf2
    geometry_msgs
    tf2_geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${orocos_kdl_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    /home/djole/youBot_driver_inst_dir/usr/local/include
    /home/djole/youBot_driver_inst_dir/usr/local/include/youbot_driver/soem
    /home/djole/youBot_driver_inst_dir/usr/local/include/youbot_driver/soem/osal
    /home/djole/youBot_driver_inst_dir/usr/local/include/youbot_driver/soem/oshw/linux
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/include
)

link_directories(
  /home/djole/youBot_driver_inst_dir/usr/local/lib
  "/home/djole/Master/Thesis/GIT/MT_testing/KDL/KDL_install_dir/lib/"
)

add_library(controller
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/constants.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/kdl_eigen_conversions.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/geometry_utils.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/moving_slope.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/moving_variance.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/model_prediction.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/finite_state_machine.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/motion_profile.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/solver_recursive_newton_euler.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/solver_vereshchagin.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/fk_vereshchagin.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/lwr_mediator.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/lwr_kdl_model.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/safety_monitor.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/abag.cpp
    /home/djole/Master/Thesis/GIT/MT_testing/Controller/src/dynamics_controller.cpp
)
target_link_libraries(controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${USE_OROCOS_LIBRARIES}
    ${orocos_kdl_LIBRARIES}
    ${kdl_parser_LIBRARIES}
    YouBotDriver
    soem
    ${Boost_FILESYSTEM_LIBRARY}
    ${Boost_SYSTEM_LIBRARY}
    controller
)

file(GLOB headers include/${PROJECT_NAME}/*.h include/${PROJECT_NAME}/*.hpp )
orocos_install_headers( FILES ${headers} )
orocos_generate_package(INCLUDE_DIRS include)

install(DIRECTORY scripts/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
 PATTERN ".svn" EXCLUDE)
 
install(DIRECTORY launch/
 DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
 PATTERN ".svn" EXCLUDE)