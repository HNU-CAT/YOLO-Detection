# Yolov11_Tensorrt_ROS: YOLOv11与TensorRT在ROS中的实现

Yolov11_Tensorrt_ROS是一个基于ROS框架的实时目标检测系统，结合了YOLOv11的高精度检测能力和TensorRT的高性能推理优化。该系统支持多种精度模式（FP32/FP16/INT8），能够在NVIDIA GPU上实现高效的目标检测任务。

## 主要特点

- **高性能推理**：利用TensorRT优化YOLOv11模型，显著提升检测速度
- **多精度支持**：支持FP32、FP16和INT8量化模式，平衡精度与速度
- **ROS集成**：完全集成ROS框架，支持图像订阅和检测结果发布
- **灵活配置**：通过配置文件轻松调整模型参数和检测阈值
- **自定义Backbone**：采用ShuffleNetV2作为骨干网络，轻量且高效

## 系统架构

系统主要包含以下几个部分：

- **ROS节点**：负责图像数据的接收和处理结果的发布
- **TensorRT引擎**：负责模型的优化和推理
- **后处理模块**：处理检测结果，包括NMS和边界框解析
- **可视化模块**：将检测结果可视化并发布

## 安装指南

### 依赖环境

- Ubuntu 18.04/20.04
- ROS Melodic/Noetic
- CUDA 10.2/11.x
- cuDNN 8.x
- TensorRT 8.x
- OpenCV 4.x
- PyTorch 1.8+

### 编译步骤

1. 创建并初始化catkin工作空间：

```bash
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/WHDworld/Yolov11_Tensorrt_ROS.git
cd ..
catkin_make
source devel/setup.bash
```

2. 下载预训练模型：

将YOLOv11预训练模型(.onnx格式)放入指定目录，并在配置文件中指定路径。

3. 配置参数：

编辑配置文件`config/config.yaml`，设置模型路径、检测阈值和其他参数。

## 使用方法

### 运行检测节点

```bash
cd Yolov11_Tensorrt_ROS
source ./start.sh
```

`start.sh`脚本会自动执行以下操作：
1. 初始化ROS环境
2. 启动RViz可视化工具并加载预设配置
3. 启动YOLOv11检测节点
4. 等待所有进程完成

### 参数配置

主要配置参数位于`config/config.yaml`文件中：

```yaml
yolov11_tensorrt_config:
  trtFile: "path/to/your/model.trt"  # TensorRT引擎文件路径,如果没有程序在第一次运行时会自动生成（需要等待较长时间，生成的文件目录在设置的output_path/onnx_model目录下）
  onnxFile: "path/to/your/model.onnx"  # ONNX模型文件路径，如果没有.onnx格式模型，可以使用目录./onnx/export.py文件将自己的模型导出为.onnx格式
  kGpuId: 0  # GPU设备ID
  kNmsThresh: 0.45  # NMS阈值
  kConfThresh: 0.25  # 置信度阈值
  kNumClass: 80  # 类别数量
  kInputH: 640  # 推理时的图像高度，需与./onnx/export.py中的imgsz保持一致
  kInputW: 640  # 推理时的图像宽度，需与./onnx/export.py中的imgsz保持一致
  kMaxNumOutputBbox: 1000  # 最大输出边界框数量
  kNumBoxElement: 7  # 每个边界框元素数量
  use_FP16_Mode: 1  # 是否使用FP16模式
  use_INT8_Mode: 0  # 是否使用INT8模式
  cacheFile: "path/to/calibration.cache"  # INT8校准缓存文件，使用use_INT8_Mode时需要设置
  calibrationDataPath: "path/to/calibration/images"  # 校准图像路径，使用use_INT8_Mode时需要设置
  vClassNames: ["person", "bicycle", "car", ...]  # 类别名称列表
```

### 发布的话题

- `/yolo_detector/detected_image`: 可视化后的检测结果图像，默认frame_id为camera_link
- `/yolo_detector/detected_bounding_boxes`: 检测到的边界框信息
- `/yolo_detector/yolo_time`: 检测耗时信息

## 性能优化

系统支持三种精度模式：

- **FP32**：完整精度模式，提供最高检测精度
- **FP16**：半精度模式，在保持较高精度的同时显著提升推理速度
- **INT8**：8位整型量化模式，极致性能优化，适合边缘设备

通过配置文件中的`use_FP16_Mode`和`use_INT8_Mode`参数可以轻松切换精度模式。

## 自定义与扩展

### 添加新类别

1. 修改配置文件中的`vClassNames`字段，添加新的类别名称
2. 使用包含新类别的数据集重新训练模型
3. 生成新的ONNX模型并更新配置文件中的路径

### 更换骨干网络

系统默认使用ShuffleNetV2作为骨干网络，如需更换其他网络：

1. 修改`scripts/module/detector.py`中的网络定义
2. 重新训练模型并导出为ONNX格式
3. 更新配置文件中的模型路径

## 故障排除

1. **CUDA/TensorRT版本不兼容**：确保CUDA、cuDNN和TensorRT版本相互兼容
2. **模型加载失败**：检查模型路径是否正确，模型格式是否符合要求
3. **推理速度慢**：尝试切换到FP16或INT8模式，或减小输入图像尺寸

## 贡献

欢迎贡献代码或提出问题。请在提交Pull Request前确保代码通过测试，并遵循项目的代码风格指南。

## 许可证

本项目采用[TODO]许可证。有关详细信息，请参阅LICENSE文件。
