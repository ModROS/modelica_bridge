# ModROS - the relay connection between Modelica and ROS
cmake_minimum_required(VERSION 2.8.3)
project(modelica_bridge)
add_compile_options(-std=c++11)

set(CATKIN_DEPS joy roscpp std_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation ${CATKIN_DEPS})
include_directories(${catkin_INCLUDE_DIRS})

add_message_files(FILES ModComm.msg)
generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime ${CATKIN_DEPS})

include(CheckIncludeFiles)
check_include_files(sys/socket.h HAVE_SYS_SOCKET_H)

if(HAVE_SYS_SOCKET_H)
  add_executable(modbridge_node src/modbridge_node.cpp)
  target_link_libraries(modbridge_node ${catkin_LIBRARIES})
  add_dependencies(modbridge_node ${modelica_bridge_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
else(HAVE_SYS_SOCKET_H)
  message("Warning: no <sys/socket.h>; won't build modelica_bridge node")
endif(HAVE_SYS_SOCKET_H)