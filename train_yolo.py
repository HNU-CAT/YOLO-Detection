from ultralytics import YOLO

# 初始化 YOLO 模型
model = YOLO('yolo11n.pt')  # 选择预训练模型，yolov8n 是最小模型

# 开始训练
model.train(
    data='dataset.yaml',     # 数据集配置文件
    epochs=500,               # 训练轮数
    batch=16,                # 批量大小
    imgsz=640,               # 输入图片大小
    device=0,                # 使用 GPU (0 表示第一块 GPU)
    name='trian'          # 训练结果保存路径
)
