#!/usr/bin/env python3
# pip install ultralytics
import os
import math
import time
import threading
import sys
import rclpy
from rclpy.node import Node
from offboard_interfaces.srv import Navigate, GetTelemetry
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

# ===== КОНФИГУРАЦИЯ =====
SPEED = 0.5
TAKEOFF_Z = 1.0
ARUCO_TARGETS = [49, 81, 51, 50, 99, 61, 58, 62, 64]  # Список ID маркеров в лабиринте
IMAGE_TOPIC = "/main_camera/image_raw" # нужно тротлинговую сюда поставить
YOLO_MODEL_PATH = "./yolov8n_ncnn_model" # Можно использовать yolo11n.pt
TARGET_CLASSES = ['orange', 'teddy bear'] # Объекты для поиска

class MazeAiController(Node):
    def __init__(self):
        super().__init__("maze_ai_navigator")

        # Инициализация ИИ
        self.get_logger().info("Загрузка YOLO...")
        self.model = YOLO(YOLO_MODEL_PATH) # Используем ncnn для лучшего тем лучше мы чем чем
        self.bridge = CvBridge()
        
        # Клиенты сервисов
        self.navigate_client = self.create_client(Navigate, "/navigate")
        self.land_client = self.create_client(Trigger, "/land")
        self.telemetry_client = self.create_client(GetTelemetry, "/get_telemetry")

        self._lock = threading.Lock()
        self._last_frame = None
        self.found_objects = set()

        # Подписка на камеру для детекции в реальном времени
        self.create_subscription(Image, IMAGE_TOPIC, self._image_callback, 10)
        
        self.wait_for_services()
        self.get_logger().info("AI Navigator готов к работе")

    def _image_callback(self, msg):
        """Распознавание объектов в каждом кадре"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._last_frame = frame

            # Запуск YOLO (stream=True для экономии памяти)
            results = self.model.predict(frame, conf=0.5, verbose=False)
            
            for r in results:
                for box in r.boxes:
                    cls_id = int(box.cls[0])
                    label = self.model.names[cls_id]

                    if label in TARGET_CLASSES and label not in self.found_objects:
                        self.get_logger().info(f"!!! ОБЪЕКТ ОБНАРУЖЕН: {label.upper()} !!!")
                        self.found_objects.add(label)
                        self.save_detection(frame, label)
        except Exception as e:
            self.get_logger().error(f"Ошибка ИИ: {e}")

    def save_detection(self, frame, label):
        """Сохранение кадра с найденным объектом"""
        os.makedirs("detections", exist_ok=True)
        path = f"detections/{label}_{int(time.time())}.jpg"
        cv2.imwrite(path, frame)
        self.get_logger().info(f"Снимок сохранен: {path}")

    def wait_for_services(self):
        for client, name in [(self.navigate_client, "Navigate"), (self.land_client, "Land")]:
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"Ожидание {name}...")

    def navigate(self, x, y, z, yaw=0.0, frame_id="body", auto_arm=False):
        """Вызов сервиса навигации"""
        req = Navigate.Request()
        req.x, req.y, req.z = float(x), float(y), float(z)
        req.yaw = float(yaw)
        req.speed = SPEED
        req.frame_id = frame_id
        req.auto_arm = auto_arm
        
        future = self.navigate_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result().success

    def run_mission(self):
        """Основная логика: Взлет -> Маркеры -> Посадка"""
        try:
            # 1. Взлет
            self.get_logger().info("Взлет...")
            if not self.navigate(0.0, 0.0, TAKEOFF_Z, frame_id="body", auto_arm=True):
                return
            time.sleep(5.0)

            # 2. Проход по ArUco-маркерам
            for aruco_id in ARUCO_TARGETS:
                marker_frame = f"aruco_{aruco_id}"
                self.get_logger().info(f"Перелет к маркеру {marker_frame}")
                
                # Точка (0,0,1) в СК маркера — это зависание в 1 метре ПЕРЕД ним
                success = self.navigate(0.0, 0.0, 1.0, yaw=0.0, frame_id=marker_frame)
                
                if success:
                    self.get_logger().info(f"Достигнут маркер {aruco_id}. Поиск объектов...")
                    time.sleep(4.0) # Даем время ИИ осмотреться
                else:
                    self.get_logger().warn(f"Не удалось найти маркер {aruco_id}, пропускаю.")

            # 3. Посадка
            self.get_logger().info("Миссия завершена. Садимся...")
            self.land_client.call_async(Trigger.Request())
            
        except Exception as e:
            self.get_logger().error(f"Критическая ошибка миссии: {e}")
            self.land_client.call_async(Trigger.Request())

def main():
    rclpy.init()
    node = MazeAiController()
    
    # Запуск миссии в отдельном потоке
    mission_thread = threading.Thread(target=node.run_mission)
    mission_thread.start()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n>>> Interrupted, landing...")
        try:
            node.land()
            time.sleep(3.0)
        except Exception:
            pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()