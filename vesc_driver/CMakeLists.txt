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
        include/vesc_driver/motor_controller_state.h
        include/vesc_driver/vesc_driver_interface.h
        src/vesc_driver_mockup.cpp include/vesc_driver/vesc_driver_mockup.h src/periodic_execution.cpp include/vesc_driver/periodic_execution.h include/vesc_driver/packet.h src/transport_request.cpp include/vesc_driver/transport_request.h include/vesc_driver/transport.h src/serial_transport.cpp include/vesc_driver/serial_transport.h src/serial_packet_codec.cpp include/vesc_driver/serial_packet_codec.h src/byte_buffer.cpp include/vesc_driver/byte_buffer.h include/vesc_driver/blocking_queue.h src/vesc_driver_serial.cpp include/vesc_driver/vesc_driver_serial.h)

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
