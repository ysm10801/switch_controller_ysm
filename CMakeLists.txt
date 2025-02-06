cmake_minimum_required(VERSION 3.0.2)
project(switch_controller)


set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c++11 -lpthread")


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  dynamic_reconfigure
  tf2
  tf2_ros
)

find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Franka 0.7.0 REQUIRED)
find_package(Threads REQUIRED)
find_package(catkin REQUIRED)
find_package(pinocchio REQUIRED)
find_package(qpSWIFT)

generate_dynamic_reconfigure_options(
  cfg/SwitchController.cfg
)

catkin_package()

include_directories(
  include/
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${pinocchio_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
)

link_libraries(Franka::Franka Threads::Threads pinocchio::pinocchio)

set(
  SRC_FILES
  src/base/base.cpp
  src/controllers/task_space_imp_ctrl.cpp
  src/controllers/task_space_NRIC_ctrl.cpp
  src/robots/franka_panda.cpp
  src/switch_controller.cpp
  src/switch_controller_node.cpp
  )

add_executable(switch_controller ${SRC_FILES})
target_link_libraries(switch_controller ${catkin_LIBRARIES} qpSWIFT::qpSWIFT-shared)
add_dependencies(switch_controller ${PROJECT_NAME}_gencfg) ##dynamic reconfig
