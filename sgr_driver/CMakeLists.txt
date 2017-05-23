cmake_minimum_required(VERSION 2.8.3)
project(sgr_driver)

# Find catkin
find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    nav_msgs
    tf
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
    DEPENDS rt pthread
)


## Sources
set(serial_SRCS
    include/serial/serial.cc
    include/serial/serial.h
    include/serial/v8stdint.h
)
list(APPEND serial_SRCS src/impl/unix.cc)
list(APPEND serial_SRCS src/impl/list_ports/list_ports_linux.cc)
add_library(${PROJECT_NAME} ${serial_SRCS})
target_link_libraries(${PROJECT_NAME} rt pthread)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(omniSerialCommunicator src/omniSerialCommunicator.cc)
add_dependencies(omniSerialCommunicator ${PROJECT_NAME})
target_link_libraries(omniSerialCommunicator ${PROJECT_NAME})
target_link_libraries(omniSerialCommunicator ${catkin_LIBRARIES})

add_executable(odom_publisher src/odom_publisher.cc)
add_dependencies(odom_publisher ${PROJECT_NAME})
target_link_libraries(odom_publisher ${PROJECT_NAME})
target_link_libraries(odom_publisher ${catkin_LIBRARIES})

add_executable(sqaure_move src/square.cc)
add_dependencies(sqaure_move ${PROJECT_NAME})
target_link_libraries(sqaure_move ${PROJECT_NAME})
target_link_libraries(sqaure_move ${catkin_LIBRARIES})


## Include headers
include_directories(include)

## Install executable
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

## Install headers
install(FILES include/serial/serial.h include/serial/v8stdint.h
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/serial)
