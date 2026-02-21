#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
import threading
from offboard_interfaces.srv import Navigate, GetTelemetry
from std_srvs.srv import Trigger
import sys
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import glob
from ultralytics import YOLO

# ============================================
# ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ ДЛЯ КОНФИГУРАЦИИ КУБА
# ============================================
CELL = 0.8
TAKEOFF_HEIGHT = 0.4       # Высота взлета в метрах
FLIGHT_SPEED = 0.5         # Скорость полета м/с
# ============================================

# сейв на всякий
BODY_MOVES = [
    ("FWD_1",   +CELL,  0.0,     0.0),
    ("FWD_2",   +CELL,  0.0,      0.0),
    ("LEFT_1",  0.0,    +CELL, 0.0),
    ("BACK_1",  -CELL,  0.0,     0.0),
    ("BACK_2",  -CELL,  0.0,     0.0),
    ("LEFT_1",  0.0,    +CELL,  0.0),
    ("FWD_1",   +CELL,  0.0,    0.0),
    ("FWD_2",   +CELL,  0.0,    0.0),
]

ARUCO_TARGETS = [49, 81, 51, 50, 61, 50, 58, 62, 64]  # Список ID маркеров в лабиринте
IMAGE_TOPIC = "/aruco_det/debug_image" # нужно тротлинговую сюда поставить
YOLO_MODEL_PATH = "./yolov8n_ncnn_model" # Можно использовать yolo11n.pt
TARGET_CLASSES = ['orange', 'teddy bear'] # Объекты для поиска

PHOTOS_DIR = "photos"
os.makedirs(PHOTOS_DIR, exist_ok=True)

DETECTIONS_DIR = "detections"
os.makedirs(DETECTIONS_DIR, exist_ok=True)

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Инициализация ИИ
        self.bridge = CvBridge()

        # Создаем клиентов для сервисов
        self.navigate_client = self.create_client(Navigate, '/navigate')
        self.land_client = self.create_client(Trigger, '/land')
        self.telemetry_client = self.create_client(GetTelemetry, '/get_telemetry')

        # Подписка на камеру для детекции в реальном времени
        self._last_frame = None
        self.found_objects = set()
        self.create_subscription(Image, IMAGE_TOPIC, self._image_callback, 10)
        
        # Ждем готовности сервисов
        self.wait_for_services()
        
        self.get_logger().info('All services are ready')
        self._lock = threading.Lock()  # Блокировка для потокобезопасности
        

    def _image_callback(self, msg):
        """Распознавание объектов в каждом кадре"""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._last_frame = frame
        except Exception as e:
            self.get_logger().error(f"Ошибка Изображения: {e}")
    
    def recognize(self):
        frame = self._last_frame
        if frame is None:
            self.get_logger().warn("recognize: no frame available")
            return
        
        # stream=True для экономии памяти
        results = self.model.predict(frame, conf=0.5, verbose=False, stream=True)

        for r in results:
            for box in r.boxes:
                cls_id = int(box.cls[0])
                label = self.model.names[cls_id]

                if label in TARGET_CLASSES and label not in self.found_objects:
                    self.get_logger().info(f"!!! ОБЪЕКТ ОБНАРУЖЕН: {label.upper()} !!!")
                    self.found_objects.add(label)

                    os.makedirs("detections", exist_ok=True)
                    path = f"detections/{label}_{int(time.time())}.jpg"
                    cv2.imwrite(path, frame)
                    self.get_logger().info(f"Снимок сохранен: {path}")

    def run_yolo_postflight(self):
        photos = sorted(glob.glob(os.path.join(PHOTOS_DIR, "scan_raw_*.jpg")))
        if not photos:
            self.get_logger().warn(f"No photos found in {PHOTOS_DIR}/ (expected scan_raw_*.jpg)")
            return

        self.get_logger().info("Loading YOLO model (postflight)...")
        try:
            model = YOLO(YOLO_MODEL_PATH)
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model '{YOLO_MODEL_PATH}': {e}")
            return

        found_summary = []  # list of (photo, labels)

        for p in photos:
            try:
                results = model.predict(source=p, conf=0.5, verbose=False)
            except Exception as e:
                self.get_logger().warn(f"YOLO failed on {p}: {e}")
                continue

            found = []
            for r in results:
                for box in getattr(r, "boxes", []):
                    cls_id = int(box.cls[0])
                    label = model.names.get(cls_id, str(cls_id))
                    if label in TARGET_CLASSES:
                        found.append(label)

            # сохранить картинку с боксами (всегда, чтобы судьям листать 1 папку)
            try:
                annotated = results[0].plot() if results else None
                if annotated is not None:
                    out_name = "DETECTED_" + os.path.basename(p)
                    out_path = os.path.join(DETECTIONS_DIR, out_name)
                    cv2.imwrite(out_path, annotated)
            except Exception as e:
                self.get_logger().warn(f"Failed to save annotated for {p}: {e}")

            if found:
                found_summary.append((os.path.basename(p), sorted(set(found))))
                self.get_logger().info(f"✅ {os.path.basename(p)}: found {sorted(set(found))}")
            else:
                self.get_logger().info(f"— {os.path.basename(p)}: empty")

        # Итог
        if found_summary:
            self.get_logger().info("==== YOLO SUMMARY ====")
            for fname, labels in found_summary:
                self.get_logger().info(f"{fname}: {', '.join(labels)}")
            self.get_logger().info(f"Annotated proofs: {DETECTIONS_DIR}/DETECTED_cell_*.jpg")
        else:
            self.get_logger().info("==== YOLO SUMMARY: nothing found ====")
            self.get_logger().info(f"Annotated proofs: {DETECTIONS_DIR}/DETECTED_cell_*.jpg")

    def snap(self):
        if self._last_frame is None:
            self.get_logger().warn("No camera frame yet, skip snap")
            return False
        timestamp = int(time.time())
        path = os.path.join(PHOTOS_DIR, f"scan_raw_{timestamp}.jpg")
        try:
            cv2.imwrite(path, self._last_frame)
            self.get_logger().info(f"📸 saved: {path}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to save photo: {e}")
            return False

    def wait_for_services(self):
        """Ждем готовности всех сервисов"""
        services = [
            (self.navigate_client, 'Navigate'),
            (self.land_client, 'Land'),
            (self.telemetry_client, 'Telemetry')
        ]
        
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit("ROS2 interrupted, exiting...")
                self.get_logger().info(f'{name} service not available, waiting...')

    def call_service(self, client, request):
        """Потокобезопасный вызов сервиса"""
        with self._lock:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {str(e)}')
                    return None
            else:
                self.get_logger().error('Service call timed out')
                return None

    def navigate(self, x, y, z, yaw=0.0, speed=0.5, auto_arm=False, frame_id="body"):
        """Отправить команду навигации"""
        request = Navigate.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        request.yaw = float(yaw)
        request.speed = float(speed)
        request.auto_arm = bool(auto_arm)
        request.angular_velocity = 0.0
        request.frame_id = str(frame_id)
        
        response = self.call_service(self.navigate_client, request)
        if response:
            self.get_logger().info(f'Navigate response: success={response.success}, message="{response.message}"')
            return response.success
        return False

    def land(self):
        """Отправить команду на посадку"""
        request = Trigger.Request()
        response = self.call_service(self.land_client, request)
        if response:
            self.get_logger().info(f'Land response: success={response.success}, message="{response.message}"')
            return response.success
        return False

    def get_telemetry(self):
        """Получить телеметрию"""
        request = GetTelemetry.Request()
        response = self.call_service(self.telemetry_client, request)
        
        if response and response.connected:
            mode = response.mode if hasattr(response, 'mode') else 'UNKNOWN'
            armed = response.armed if hasattr(response, 'armed') else False
            telemetry_str = f"Telemetry: mode={mode}, armed={armed}, "
            telemetry_str += f"x={response.x:.3f}, y={response.y:.3f}, z={response.z:.3f}, "
            telemetry_str += f"yaw={response.yaw:.3f}, vx={response.vx:.3f}, vy={response.vy:.3f}, vz={response.vz:.3f}, "
            telemetry_str += f"voltage={response.voltage:.2f}V"
            self.get_logger().info(telemetry_str)
            
            return {
                'x': response.x,
                'y': response.y,
                'z': response.z,
                'yaw': response.yaw,
                'vx': response.vx,
                'vy': response.vy,
                'vz': response.vz,
                'mode': mode,
                'armed': armed,
                'voltage': response.voltage,
                'connected': True
            }
        else:
            self.get_logger().warn("Drone not connected or telemetry failed")
            return {'connected': False}

def telemetry_loop(node):
    """Поток для периодического получения телеметрии"""
    while rclpy.ok():
        try:
            node.get_telemetry()
            time.sleep(1.0)  # Получаем телеметрию раз в секунду
        except Exception as e:
            node.get_logger().error(f'Telemetry error: {str(e)}')
            time.sleep(2.0)  # Ждем дольше при ошибке

def main():
    rclpy.init()
    drone = DroneController()
    
    # Запускаем поток телеметрии
    telemetry_thread = threading.Thread(target=lambda: telemetry_loop(drone), daemon=True)
    telemetry_thread.start()
    
    try:
        print("\n=== Drone Control Script Started ===")
        print("Press Ctrl+C to stop at any time\n")
        
        # Ждем немного для стабилизации
        time.sleep(2)
        
        # 1. Взлет на заданную высоту
        print(f"\n>>> Taking off to {TAKEOFF_HEIGHT} meters")
        success = drone.navigate(0.0, 0.0, TAKEOFF_HEIGHT, yaw=0.0, speed=1.0, auto_arm=True, frame_id="body")
        if not success:
            print("ERROR: Takeoff failed")
            return
        
        # Ждем завершения взлета
        print("Waiting for takeoff to complete...")
        time.sleep(5)

        # Полёт по змейке (relative body)
        for i, (name, dx, dy, dz) in enumerate(BODY_MOVES, start=1):
            success = drone.navigate(dx, dy, dz, yaw=0.0, speed=FLIGHT_SPEED, auto_arm=False, frame_id="body")

            if success:
                print(f"Двигаюсь к {name}")
                drone.snap()
                #drone.recognize()
                time.sleep(4.0)
            else:
                print(f"чет хуйня какая-то на {name}")

        """
        # 2. Полет по меткам
        print("Полет по меткам")

        for aruco_id in ARUCO_TARGETS:
            marker_frame = f"aruco_{aruco_id}"
            print(f"Перелет к маркеру {marker_frame}")
            
            # Точка (0,0,1) в СК маркера — это зависание в 1 метре ПЕРЕД ним
            success = drone.navigate(x=0.0, y=0.0, z=0.3, yaw=0.0, speed=FLIGHT_SPEED, frame_id=marker_frame)
            
            if success:
                print(f"Достигнут маркер {aruco_id}. Поиск объектов...")
                drone.snap(f"{aruco_id} + {int(time.time())}")
                drone.recognize()
                time.sleep(3.0) # Даем время ИИ осмотреться
            else:
                print(f"Не удалось найти маркер {aruco_id}, пропускаю.")
        """
            
        # 3. Посадка
        print("\n>>> Landing")
        success = drone.land()
        if success:
            print("Landing command sent successfully")
            print("Waiting for landing to complete...")
            drone.run_yolo_postflight()
            time.sleep(10)  # Ждем завершения посадки
        else:
            print("ERROR: Landing command failed")
            
        print("\n=== Flight completed successfully ===")
        
    except KeyboardInterrupt:
        print("\n>>> Operation interrupted by user")
        print("Attempting to land...")
        try:
            drone.land()
            drone.run_yolo_postflight()
            time.sleep(5)
        except:
            pass
        
    except Exception as e:
        print(f"ERROR: Unexpected error occurred: {str(e)}")
        print("Attempting emergency landing...")
        try:
            drone.land()
        except:
            pass
            
    finally:
        # Завершаем работу
        print("\nShutting down...")
        drone.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()