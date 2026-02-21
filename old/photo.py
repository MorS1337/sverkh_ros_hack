#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# ============================================
# НАСТРОЙКИ
# ============================================
IMAGE_TOPIC = "/aruco_det/debug_image" 
YOLO_MODEL_PATH = "./yolov8n_ncnn_model" 
TARGET_CLASSES = ['orange', 'teddy bear'] 

PHOTOS_DIR = "photos"
DETECTIONS_DIR = "detections"
# ============================================

class ImageScannerNode(Node):
    def __init__(self):
        super().__init__('image_scanner')

        # Создаем папки, если их нет
        os.makedirs(PHOTOS_DIR, exist_ok=True)
        os.makedirs(DETECTIONS_DIR, exist_ok=True)

        self.get_logger().info("Загрузка YOLO...")
        self.model = YOLO(YOLO_MODEL_PATH)
        self.bridge = CvBridge()
        
        self.get_logger().info(f"Ожидание изображения из топика {IMAGE_TOPIC}...")
        
        # Подписываемся на камеру
        self.subscription = self.create_subscription(
            Image, 
            IMAGE_TOPIC, 
            self._image_callback, 
            10
        )
        self.image_processed = False

    def _image_callback(self, msg):
        # Если уже сделали фотку, игнорируем новые кадры
        if self.image_processed:
            return 

        self.get_logger().info("📸 Изображение получено! Начинаю распознавание...")
        
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Ошибка конвертации изображения: {e}")
            return

        timestamp = int(time.time())

        # 1. Сохраняем исходное фото
        raw_path = os.path.join(PHOTOS_DIR, f"scan_raw_{timestamp}.jpg")
        cv2.imwrite(raw_path, frame)
        self.get_logger().info(f"Исходное фото сохранено: {raw_path}")

        # 2. Запускаем распознавание
        results = self.model.predict(frame, conf=0.5, verbose=False)
        found_targets = []

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                if label in TARGET_CLASSES:
                    found_targets.append(label)
                    self.get_logger().info(f"!!! ОБЪЕКТ ОБНАРУЖЕН: {label.upper()} !!!")
        
        # 3. Сохраняем результат с рамками (bounding boxes), если что-то нашли
        if found_targets:
            # .plot() автоматически рисует рамки вокруг найденных объектов
            annotated_frame = results[0].plot() 
            det_path = os.path.join(DETECTIONS_DIR, f"scan_detected_{timestamp}.jpg")
            cv2.imwrite(det_path, annotated_frame)
            self.get_logger().info(f"Снимок с рамками объектов сохранен: {det_path}")
        else:
            self.get_logger().info("Целевые объекты на кадре не найдены.")

        # Ставим флаг, что всё готово, чтобы скрипт мог завершиться
        self.image_processed = True

def main(args=None):
    rclpy.init(args=args)
    scanner = ImageScannerNode()
    
    try:
        # Крутим ноду (ожидаем сообщения), пока кадр не будет обработан
        while rclpy.ok() and not scanner.image_processed:
            rclpy.spin_once(scanner, timeout_sec=0.1)
            
        print("\n=== Сканирование успешно завершено ===")
        
    except KeyboardInterrupt:
        print("\n>>> Прервано пользователем")
    finally:
        scanner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()