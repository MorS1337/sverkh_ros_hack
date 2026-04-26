# Просто пакеты
import os, glob

# Для камеры
import cv2
from ultralytics import YOLO

YOLO_MODEL_PATH = "./yolov8n_ncnn_model"
TARGET_CLASSES = ['orange', 'teddy bear']
PHOTOS_DIR = "photos"
RESULTS_DIR = "detections"

class RecognizeImage:
    def __init__(self):
        pass

    def start(self):
        photos = sorted(glob.glob(os.path.join(PHOTOS_DIR, "shot*.jpg")))

        print("Loading YOLO...")
        model = YOLO(YOLO_MODEL_PATH)

        print("YOLO loaded, starting predict...")
        for p in photos:
            # Запускаем распознование 
            results = model.predict(p, conf=0.1, verbose=False)
            found_targets = []

            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    label = model.names[cls_id]

                    if label in TARGET_CLASSES:
                        found_targets.append(label)
                        print(f"!!! ОБЪЕКТ ОБНАРУЖЕН: {label.upper()} !!!")
            
            if found_targets:
                # .plot() автоматически рисует рамки вокруг найденных объектов
                annotated_frame = results[0].plot() 
                det_path = os.path.join(RESULTS_DIR, "found_" + os.path.basename(p))
                cv2.imwrite(det_path, annotated_frame)
                print(f"Снимок с рамками объектов сохранен: {det_path}")
            else:
                print("Целевые объекты на кадре не найдены.")

        print("All images recognized.")


