from ultralytics import YOLO

# Path to Roboflow data.yaml
DATA_YAML = "datasets/bottles.v2i.yolov8-obb/data.yaml"

# YOLO OBB model — you can use n/s/m/l/x
model = YOLO("yolov8n-obb.pt")

model.train(
    data=DATA_YAML,
    epochs=200,
    imgsz=1024,
    batch=8,
    name="bottle_obb_train"
)
