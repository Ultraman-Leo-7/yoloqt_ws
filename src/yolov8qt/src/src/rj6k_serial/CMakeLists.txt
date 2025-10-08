cmake_minimum_required(VERSION 3.0.2)
project(rj6k_serial)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  serial
  message_generation
)

add_message_files(FILES RJ6KData.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rj6k_serial
  CATKIN_DEPENDS roscpp std_msgs serial message_runtime
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(read_sensor src/read_sensor.cpp)
target_link_libraries(read_sensor ${catkin_LIBRARIES})

add_executable(pub_rj6k src/pub_rj6k.cpp)
target_link_libraries(pub_rj6k ${catkin_LIBRARIES})