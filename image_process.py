import os
import glob
import cv2
from ultralytics import YOLO

OUTPUT_DIR = "results"
os.makedirs(OUTPUT_DIR, exist_ok=True)

print("Загрузка модели YOLOv8...")
model = YOLO('yolov8n.pt') 

TARGET_CLASSES = ['orange', 'teddy bear']

image_files = glob.glob("photos/cell_*.jpg")
if not image_files:
    print("❌ Фотки не найдены! Убедись, что скрипт лежит в той же папке.")
    exit()

print(f"Найдено фотографий: {len(image_files)}. Начинаем инференс...\n")

total_score = 0

for img_path in image_files:
    filename = os.path.basename(img_path)
    results = model.predict(source=img_path, conf=0.4, verbose=False)
    
    found_targets = []
    
    for r in results:
        boxes = r.boxes
        for box in boxes:
            class_id = int(box.cls[0])
            class_name = model.names[class_id]
            
            if class_name in TARGET_CLASSES:
                found_targets.append(class_name)
    
    if found_targets:
        targets_str = ", ".join(found_targets).upper()
        print(f"В файле {filename} найдено: {targets_str}")
        total_score += 100 * len(found_targets)
        plotted_img = results[0].plot()
        
        save_path = os.path.join(OUTPUT_DIR, f"DETECTED_{filename}")
        cv2.imwrite(save_path, plotted_img)
    else:
        print(f"В файле {filename} пусто.")

print("\n" + "="*40)
print(f"Пруфы с рамками сохранены в папку: {OUTPUT_DIR}/")
print("="*40)