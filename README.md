# YOLO-Detection 🚁👀

本项目通过结合深度图和彩色图，使用 YOLO（You Only Look Once）算法进行行人检测，并进一步预测其三维位置。该系统适用于无人机的目标检测与追踪，提供了高效且实时的处理能力。

## 功能概述 ⚙️

1. **YOLO 行人检测** 🏃‍♂️：利用 YOLO 网络进行目标检测，识别图像中的行人。
2. **深度图处理** 🌌：根据深度图获取目标的空间信息，进一步提升目标定位的精度。
3. **三维位置预测** 🗺️：通过结合 YOLO 检测结果与深度信息，预测目标的三维空间位置。
4. **追踪功能** 🎯：对检测到的目标进行追踪，确保目标在动态环境下的持续监测。

## 项目结构 📂

```
.
├── src
│   ├── detection.cpp        # YOLO 行人检测代码
│   ├── tracker.cpp         # 目标追踪模块
│   ├── ekf_predictor.cpp   # EKF 预测模块
│   └── main.cpp            # 主程序入口
├── launch
│   └── detection.launch    # 启动文件
├── README.md               # 项目说明文档
└── CMakeLists.txt          # CMake 构建文件
```

## 安装与配置 🛠️

### 1. 安装依赖 🔧

本项目依赖于 ROS 和一些外部库，您需要确保已安装以下软件：
- ROS (建议使用 ROS Noetic 或更高版本) 🐢
- OpenCV 📷
- Eigen 📐
- cv_bridge 🔄
- FastestDet 模型文件（根据需要进行训练或下载）

### 2. 设置环境 🌍

在启动项目之前，确保设置好 ROS 环境变量。通过以下命令配置：

```bash
source /opt/ros/noetic/setup.bash
source devel/setup.bash
```

### 3. 构建项目 🔨

在工作空间内构建项目：

```bash
catkin_make
```

### 4. 启动项目 🚀

您可以通过以下命令启动 YOLO 行人检测节点：

```bash
roslaunch <your_package> detection.launch
```

## TODO 📋

1. **训练无人机的检测模型** 🚁：为无人机目标识别训练适应性的模型。
2. **完善 README 说明文档** 📚：详细说明各模块的使用方法与参数。
3. **测试目前用于追踪的效果** 🎯：优化追踪算法，确保高效性和准确性。
4. **迁移到建图模块中** 🗺️：将检测和追踪功能集成到建图与定位模块中，提升系统性能。
5. **EKF 的预测中** 📊：结合扩展卡尔曼滤波（EKF）算法进行目标状态的预测，提升跟踪精度。

## 贡献 🤝

欢迎对本项目进行贡献！请提交 issue 或 pull request 提出您的想法。