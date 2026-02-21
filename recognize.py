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
            try:
                results = model.predict(source=p, conf=0.5, verbose=False)
            except Exception as e:
                print(f"YOLO failed on {p}: {e}")
                continue

            found = set()
            for r in results:
                for box in getattr(r, "boxes", []):
                    cls_id = int(box.cls[0])
                    name = model.names.get(cls_id, str(cls_id))
                    if name in TARGET_CLASSES:
                        found.add(name)

            try:
                annotated = results[0].plot() if results else None
                if annotated is not None:
                    out = os.path.join(RESULTS_DIR, "found_" + os.path.basename(p))
                    cv2.imwrite(out, annotated)
            except Exception:
                pass

            if found:
                print(f"✅ {os.path.basename(p)} -> {sorted(found)}")
            else:
                print(f"— {os.path.basename(p)} -> empty")

        print("All images recognized.")


