
# YOLO-Detection

使用YOLO和深度图获得目标位置的三维位置

![YOLO检测示例](https://github.com/user-attachments/assets/e6ff846b-af22-4c0c-a3eb-62df64911c1e#gh-light-mode-only)
![YOLO检测示例](https://github.com/user-attachments/assets/e6ff846b-af22-4c0c-a3eb-62df64911c1e#gh-dark-mode-only)

YOLO部分使用[tensorrt](https://github.com/HNU-CAT/yolov11_tensorrt_ros)加速

![TensorRT加速效果](https://github.com/user-attachments/assets/be53bd4f-44f6-49f5-9911-e96f77e61691#gh-light-mode-only)
![TensorRT加速效果](https://github.com/user-attachments/assets/be53bd4f-44f6-49f5-9911-e96f77e61691#gh-dark-mode-only)

检测部分参考[onboard-detect](https://github.com/Zhefan-Xu/onboard_detector)，使用MAD计算BBOX中的深度，从而获得更好的效果。

## 主要特点
- 基于YOLOv11的实时目标检测
- 利用TensorRT加速推理过程
- 结合深度图计算目标的三维空间位置
- 使用MAD算法优化深度计算精度
