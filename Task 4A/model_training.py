from ultralytics import YOLO

model = YOLO("yolov8m.pt")  # load a pretrained model (recommended for training)
model.train(data="dataset_4a/data.yaml", epochs=100,imgsz=640)