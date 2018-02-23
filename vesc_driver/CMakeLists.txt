cmake_minimum_required(VERSION 2.8.3)
project(vesc_driver)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  pluginlib
  roscpp
  serial
  std_msgs
  vesc_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS nodelet pluginlib roscpp serial std_msgs vesc_msgs
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_ros
)

###########
## Build ##
###########

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/${PROJECT_NAME}/datatypes.h
  include/${PROJECT_NAME}/v8stdint.h
  src/vesc_driver.cpp include/${PROJECT_NAME}/vesc_driver.h
  src/vesc_interface.cpp include/${PROJECT_NAME}/vesc_interface.h
  src/vesc_packet.cpp include/${PROJECT_NAME}/vesc_packet.h
  src/vesc_packet_factory.cpp include/${PROJECT_NAME}/vesc_packet_factory.h
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

add_library(${PROJECT_NAME}_ros
  src/vesc_ros_driver.cpp include/${PROJECT_NAME}/vesc_ros_driver.h
)

target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# node executable
add_executable(${PROJECT_NAME}_node
  src/vesc_driver_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_ros
  ${catkin_LIBRARIES}
)

# nodelet library
add_library(${PROJECT_NAME}_nodelet
  src/vesc_driver_nodelet.cpp
)

target_link_libraries(${PROJECT_NAME}_nodelet
  ${PROJECT_NAME}_ros
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ros ${PROJECT_NAME}_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES vesc_driver_nodelet.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

#############
## Testing ##
#############

# TODO
