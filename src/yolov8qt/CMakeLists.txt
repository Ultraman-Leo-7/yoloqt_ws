# cmake_minimum_required(VERSION 3.1)

# # cmake_policy(SET CMP0074 NEW)

# set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86 89 90)
# set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

# project(yolov8qt)

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_BUILD_TYPE Release)
# option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)


# # CUDA
# # 设置CMP0074策略为NEW，让find_package使用<PackageName>_ROOT变量
# #cmake_policy(SET CMP0074 NEW)

# find_package(CUDA REQUIRED)
# message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
# get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
# message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# # OpenCV
# # set(OpenCV_DIR /home/icebear/opencv/opencv-4.2.0/build)
# # set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
# find_package(OpenCV REQUIRED)
# message(STATUS "OpenCV Libs: \n${OpenCV_LIBS}\n")
# message(STATUS "OpenCV Libraries: \n${OpenCV_LIBRARIES}\n")
# message(STATUS "OpenCV Headers: \n${OpenCV_INCLUDE_DIRS}\n")


# # PCL
# find_package(PCL REQUIRED COMPONENTS common io visualization)
 
# # 添加消息生成指令
# add_message_files(FILES RJ6KData.msg)
# generate_messages(DEPENDENCIES std_msgs)

# catkin_package(
#   CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport pcl_ros message_filters serial message_runtime
# )


# # Catkin
# find_package(catkin REQUIRED COMPONENTS
#   roscpp
#   roslib
#   rospy
#   pcl_ros
#   cv_bridge
#   sensor_msgs
#   std_msgs
#   message_filters
#   image_transport
#   message_generation
#   serial
# )



# # Eigen
# find_package(Eigen3 REQUIRED)

# # TensorRT
# set(TensorRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)
# set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)

# message(STATUS "TensorRT Libs: \n${TensorRT_LIBRARIES}\n")
# message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")

# list(APPEND INCLUDE_DIRS
#   ${CUDA_INCLUDE_DIRS}
#   ${PCL_INCLUDE_DIRS}
#   ${OpenCV_INCLUDE_DIRS}
#   ${TensorRT_INCLUDE_DIRS}
#   ${catkin_INCLUDE_DIRS}
#   ${EIGEN3_INCLUDE_DIR}
#   include
# )

# list(APPEND ALL_LIBS
#   ${PCL_LIBRARIES}
#   ${OpenCV_LIBRARIES}  
#   ${OpenCV_LIBS}
#   ${CUDA_LIBRARIES}
#   ${CUDA_LIB_DIR}
#   ${TensorRT_LIBRARIES}
#   ${TensorRT_LIBRARIES}/libnvinfer.so
#   ${TensorRT_LIBRARIES}/libnvinfer_plugin.so
# )

# include_directories(${INCLUDE_DIRS})

# ##############################################################################
# # Qt Environment
# ##############################################################################

# find_package(Qt5 REQUIRED Core Widgets)
# set(QT_LIBRARIES Qt5::Widgets)


# ##############################################################################
# # Sections
# ##############################################################################

# file(GLOB QT_FORMS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} ui/mainwindow.ui)
# file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
# file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/yolov8qt/yolov8.h common.hpp ClickableLabel.h project.h QNode.h mainwindow.h electric_field.h)

# QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
# QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
# QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

# ##############################################################################
# # Sources
# ##############################################################################

# file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/ClickableLabel.cpp yolov8.cpp project.cpp QNode.cpp mainwindow.cpp electric_field.cpp pub_rj6k.cpp main.cpp)

# ##############################################################################
# # Binaries
# ##############################################################################

# # set(CMAKE_SKIP_RPATH TRUE)

# set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
# set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# # 单独的 pub_rj6k 节点
# add_executable(pub_rj6k src/pub_rj6k.cpp)
# target_link_libraries(pub_rj6k ${catkin_LIBRARIES})

# add_executable(yolov8qt ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
# target_link_libraries(yolov8qt ${QT_LIBRARIES} ${catkin_LIBRARIES} ${ALL_LIBS})


# # install(TARGETS yolov8qt RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# install(TARGETS yolov8qt pub_rj6k RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})




cmake_minimum_required(VERSION 3.1)

# 设置 CUDA 架构和编译器
set(CMAKE_CUDA_ARCHITECTURES 60 61 62 70 72 75 86 89 90)
set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)

# 设置项目名称
project(yolov8qt)

# 设置 C++ 标准和优化选项
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
option(CUDA_USE_STATIC_CUDA_RUNTIME OFF)

# 加载 catkin 和其他依赖项
find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  rospy
  pcl_ros
  cv_bridge
  sensor_msgs
  std_msgs
  message_filters
  image_transport
  message_generation
  serial
)

# 添加自定义消息
add_message_files(
  FILES
  RJ6KData.msg
)

# 生成消息
generate_messages(
  DEPENDENCIES
  std_msgs
)

# 加载 CUDA
find_package(CUDA REQUIRED)
message(STATUS "CUDA Libs: \n${CUDA_LIBRARIES}\n")
get_filename_component(CUDA_LIB_DIR ${CUDA_LIBRARIES} DIRECTORY)
message(STATUS "CUDA Headers: \n${CUDA_INCLUDE_DIRS}\n")

# 加载 OpenCV
find_package(OpenCV REQUIRED)
message(STATUS "OpenCV Libs: \n${OpenCV_LIBS}\n")
message(STATUS "OpenCV Libraries: \n${OpenCV_LIBRARIES}\n")
message(STATUS "OpenCV Headers: \n${OpenCV_INCLUDE_DIRS}\n")

# 加载 PCL
find_package(PCL REQUIRED COMPONENTS common io visualization)

# 加载 Eigen
find_package(Eigen3 REQUIRED)

# 设置 TensorRT
set(TensorRT_INCLUDE_DIRS /usr/include/aarch64-linux-gnu)
set(TensorRT_LIBRARIES /usr/lib/aarch64-linux-gnu)
message(STATUS "TensorRT Libs: \n${TensorRT_LIBRARIES}\n")
message(STATUS "TensorRT Headers: \n${TensorRT_INCLUDE_DIRS}\n")

# 设置包含目录
list(APPEND INCLUDE_DIRS
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${TensorRT_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  include
)

# 设置链接库
list(APPEND ALL_LIBS
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}  
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${CUDA_LIB_DIR}
  ${TensorRT_LIBRARIES}
  ${TensorRT_LIBRARIES}/libnvinfer.so
  ${TensorRT_LIBRARIES}/libnvinfer_plugin.so
)

# 包含目录
include_directories(${INCLUDE_DIRS})

# Qt 环境
find_package(Qt5 REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

# Qt 文件
set(QT_FORMS
  ${CMAKE_CURRENT_SOURCE_DIR}/ui/mainwindow.ui
)
file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)
file(GLOB_RECURSE QT_MOC RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS include/yolov8qt/yolov8.h common.hpp ClickableLabel.h project.h QNode.h mainwindow.h electric_field.h)

# Qt 资源和 UI 文件
QT5_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT5_WRAP_UI(QT_FORMS_HPP ${QT_FORMS})
QT5_WRAP_CPP(QT_MOC_HPP ${QT_MOC})

# 源文件
file(GLOB_RECURSE QT_SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} FOLLOW_SYMLINKS src/ClickableLabel.cpp yolov8.cpp project.cpp QNode.cpp mainwindow.cpp electric_field.cpp main.cpp)

# 二进制文件
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# 单独的 pub_rj6k 节点
add_executable(pub_rj6k src/pub_rj6k.cpp)
target_link_libraries(pub_rj6k ${catkin_LIBRARIES})

# 主程序 yolov8qt
add_executable(yolov8qt ${QT_SOURCES} ${QT_RESOURCES_CPP} ${QT_FORMS_HPP} ${QT_MOC_HPP})
target_include_directories(yolov8qt PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(yolov8qt ${QT_LIBRARIES} ${catkin_LIBRARIES} ${ALL_LIBS})

# 安装目标
install(TARGETS yolov8qt pub_rj6k RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# 包依赖
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs cv_bridge image_transport pcl_ros message_filters serial message_runtime
)