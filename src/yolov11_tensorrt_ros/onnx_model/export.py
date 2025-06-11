from ultralytics import YOLO
import time

# Load a YOLO11n PyTorch model
model = YOLO("./yolo11n.pt")


# Export the model to TensorRT
model.export(
    imgsz=(640,640),
    dynamic=False,
    simplify=True,
    format="onnx", 
    device="0", 
    half=False)
