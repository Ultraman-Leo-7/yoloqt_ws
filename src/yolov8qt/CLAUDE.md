# CLAUDE.md

本文件为 Claude Code (claude.ai/code) 提供在此代码库工作时的指导。

## 项目概述

这是一个基于 YOLOv8 的目标检测系统的 ROS Catkin 工作空间，集成了 Qt 图形界面，专为变电站设备监测设计。系统结合了计算机视觉（YOLOv8）、三维点云处理和电场测量，实现对设备状态的综合分析。

## 系统架构

### 核心组件

1. **YOLOv8 检测引擎** (`yolov8.cpp`, `yolov8.h`)

   - 在 NVIDIA Jetson 平台上使用 TensorRT 加速推理
   - 实时处理相机图像的目标检测
   - 支持通过 GUI 切换多个模型
2. **Qt 图形界面应用** (`mainwindow.cpp`, `mainwindow.h`)

   - 主要的可视化和控制界面
   - 显示带边界框的检测结果
   - 显示 FPS、置信度、距离和电场强度测量值
   - 允许模型选择和数据保存
3. **ROS 节点集成** (`QNode.cpp`, `QNode.h`)

   - 订阅同步的图像和点云话题
   - 订阅 RJ6K 电场测量数据
   - 发布检测结果和处理后的图像
   - 处理来自 GUI 的点击选择
4. **电场计算模块** (`electric_field.cpp`, `electric_field.h`)

   - 模拟导线和开关的电场分布
   - 从 `config/` 目录读取配置文件
   - 与实时测量数据集成
5. **RJ6K 数据发布器** (`pub_rj6k.cpp`)

   - 独立节点，用于与 RJ6K 设备的串口通信
   - 发布电场测量数据作为自定义 ROS 消息

## 编译命令

```bash
# 导航到工作空间根目录
cd ~/yoloqt_ws

# 加载 ROS 环境
source /opt/ros/noetic/setup.bash  # 或 melodic/kinetic，取决于 ROS 版本

# 编译工作空间
catkin_make

# 加载工作空间
source devel/setup.bash
```

## 运行系统

```bash
# 终端 1：启动 ROS 核心
roscore

# 终端 2：启动主 YOLOv8 Qt 应用程序
rosrun yolov8qt yolov8qt

# 终端 3（可选）：启动 RJ6K 数据发布器用于电场测量
rosrun yolov8qt pub_rj6k
```

## 主要依赖

- **ROS 包**: roscpp, sensor_msgs, cv_bridge, pcl_ros, message_filters, serial
- **深度学习**: CUDA, TensorRT（用于 Jetson 平台）
- **计算机视觉**: OpenCV 4.x
- **点云处理**: PCL (Point Cloud Library)
- **图形界面**: Qt5
- **数学库**: Eigen3

## 配置文件

- `config/powerline_config.txt`: 导线几何参数和电压参数
- `config/kaiguan_config.txt`: 开关设备参数
- `config/initial_params.txt`: 系统初始化参数
- `config/cloud_config.txt`: 点云处理设置

## 自定义 ROS 消息

- `msg/RJ6KData.msg`: RJ6K 设备的电场测量数据

## 重要话题

- **输入话题**:

  - `/camera/color/image_raw`: RGB 相机图像
  - `/camera/depth/points`: 深度相机的三维点云
  - `/rj6k_data`: 电场测量数据
- **输出话题**:

  - `/yolov8/detection_image`: 带边界框的处理后图像
  - `/yolov8/bounding_boxes`: 检测结果
  - `/yolov8/obj_height`: 目标高度计算

## 模型管理

YOLOv8 模型应放置在相应目录中，可通过 GUI 下拉菜单在运行时切换。系统在 Jetson 平台上使用 TensorRT 引擎文件（.engine）进行优化推理。

## 开发说明

- 系统专为 ARM64 架构设计（Jetson Nano/Xavier）
- 点云处理使用基于点击的选择来计算距离
- 电场模拟与检测并行运行
- 与 RJ6K 设备的串口通信运行在 9600 波特率，端口为 `/dev/ttyUSB0`
