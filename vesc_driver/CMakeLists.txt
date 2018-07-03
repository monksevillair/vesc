cmake_minimum_required(VERSION 2.8.3)
project(vesc_driver)

find_package(catkin REQUIRED COMPONENTS
  pluginlib
  roscpp
  serial
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS pluginlib roscpp serial std_msgs
  LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

add_compile_options(-std=c++11)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/vesc_driver/blocking_queue.h
  include/vesc_driver/motor_controller_state.h
  include/vesc_driver/packet.h
  include/vesc_driver/periodic_task.h src/periodic_task.cpp
  include/vesc_driver/serial_data_reader.h src/serial_data_reader.cpp
  include/vesc_driver/serial_data_writer.h src/serial_data_writer.cpp
  include/vesc_driver/serial_packet_codec.h src/serial_packet_codec.cpp
  include/vesc_driver/serial_transport.h src/serial_transport.cpp
  include/vesc_driver/transport.h
  include/vesc_driver/transport_request.h src/transport_request.cpp
  include/vesc_driver/vesc_driver_impl.h src/vesc_driver_impl.cpp
  include/vesc_driver/vesc_driver_interface.h src/vesc_driver_interface.cpp
  include/vesc_driver/vesc_driver_mockup.h src/vesc_driver_mockup.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

#############
## Testing ##
#############

# TODO
