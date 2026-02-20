#!/usr/bin/env python3
import os
import math
import time
import threading
import sys

import rclpy
from rclpy.node import Node

from offboard_interfaces.srv import Navigate, GetTelemetry
from std_srvs.srv import Trigger

CAMERA_AVAILABLE = True
try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
except Exception:
    CAMERA_AVAILABLE = False

# ==========================================
# КОНФИГ
# ==========================================
CELL = 0.8          # шаг сетки (м)
TAKEOFF_Z = 1.5     # взлёт повыше чтобы уверенно поймать aruco_map
WORK_Z = 0.8        # рабочая высота на клетках
CENTER_Z = 1.4      # центр выше (нет метки) -> лучше видим соседние маркеры

SPEED = 0.6
TOL = 0.20          # точность обычных точек
CENTER_TOL = 0.30   # точность центра
WAIT_TIMEOUT = 20.0 # таймаут ожидания достижения точки

ARUCO_MAP_FRAME = "aruco_map"
BODY_FRAME = "body"

PHOTOS_DIR = "photos"
os.makedirs(PHOTOS_DIR, exist_ok=True)

CAMERA_SENSOR = '0v5647'
IMAGE_TOPIC = f"/{CAMERA_SENSOR}/image_raw"   

# ==========================================

def is_finite(x: float) -> bool:
    return math.isfinite(x) and not math.isnan(x)

class DroneController(Node):
    def __init__(self):
        super().__init__('snake_mission')

        # сервисы
        self.navigate_client = self.create_client(Navigate, '/navigate')
        self.land_client = self.create_client(Trigger, '/land')
        self.telemetry_client = self.create_client(GetTelemetry, '/get_telemetry')

        self._lock = threading.Lock()
        self.wait_for_services()

        # камера
        self.bridge = CvBridge() if CAMERA_AVAILABLE else None
        self._last_frame = None
        self._last_frame_ts = 0.0

        if CAMERA_AVAILABLE:
            self.create_subscription(Image, IMAGE_TOPIC, self._image_cb, 10)
            self.get_logger().info(f"Camera subscription ON: {IMAGE_TOPIC}")
        else:
            self.get_logger().warn("Camera disabled (no cv_bridge/sensor_msgs/cv2).")

        self.get_logger().info("All services are ready")

    def _image_cb(self, msg: Image):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self._last_frame = frame
            self._last_frame_ts = time.time()
        except Exception:
            pass

    def wait_for_services(self):
        services = [
            (self.navigate_client, 'Navigate'),
            (self.land_client, 'Land'),
            (self.telemetry_client, 'Telemetry'),
        ]
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit("ROS2 interrupted, exiting...")
                self.get_logger().info(f'{name} service not available, waiting...')

    def call_service(self, client, request, timeout_sec=5.0):
        with self._lock:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if future.done():
                try:
                    return future.result()
                except Exception as e:
                    self.get_logger().error(f'Service call failed: {str(e)}')
                    return None
            else:
                self.get_logger().error('Service call timed out')
                return None

    def navigate(self, x, y, z, yaw=0.0, speed=SPEED, auto_arm=False, frame_id=BODY_FRAME):
        req = Navigate.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        req.speed = float(speed)
        req.angular_velocity = 1.0
        req.auto_arm = bool(auto_arm)
        req.frame_id = str(frame_id)

        resp = self.call_service(self.navigate_client, req, timeout_sec=5.0)
        if resp:
            self.get_logger().info(f'Navigate: success={resp.success} msg="{resp.message}"')
            return resp.success
        return False

    def land(self):
        req = Trigger.Request()
        resp = self.call_service(self.land_client, req, timeout_sec=5.0)
        if resp:
            self.get_logger().info(f'Land: success={resp.success} msg="{resp.message}"')
            return resp.success
        return False

    def get_telemetry(self, frame_id=None):
        req = GetTelemetry.Request()
        if frame_id is not None and hasattr(req, "frame_id"):
            req.frame_id = str(frame_id)

        resp = self.call_service(self.telemetry_client, req, timeout_sec=3.0)
        if resp and getattr(resp, "connected", False):
            return {
                "connected": True,
                "x": resp.x, "y": resp.y, "z": resp.z, "yaw": resp.yaw,
                "vx": resp.vx, "vy": resp.vy, "vz": resp.vz,
                "armed": getattr(resp, "armed", False),
                "mode": getattr(resp, "mode", "UNKNOWN"),
                "voltage": getattr(resp, "voltage", float("nan")),
            }
        return {"connected": False}

    def sleep_spin(self, duration_s: float, step=0.1):
        """сон, но с прокруткой callback'ов (чтобы камера жила)"""
        t0 = time.time()
        while time.time() - t0 < duration_s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=step)

    def wait_for_aruco_map(self, timeout_s=10.0):
        t0 = time.time()
        while time.time() - t0 < timeout_s and rclpy.ok():
            telem = self.get_telemetry(frame_id=ARUCO_MAP_FRAME)
            if telem.get("connected") and is_finite(telem["x"]) and is_finite(telem["y"]):
                return True
            self.sleep_spin(0.2)
        return False

    def ensure_aruco_map(self):
        """Если aruco_map отвалился — поднимаемся повыше по body и ждём."""
        if self.wait_for_aruco_map(timeout_s=1.5):
            return True
        self.get_logger().warn("aruco_map пропал -> поднимаюсь для восстановления...")
        self.navigate(0.0, 0.0, TAKEOFF_Z, yaw=0.0, speed=1.0, auto_arm=False, frame_id=BODY_FRAME)
        self.sleep_spin(1.0)
        return self.wait_for_aruco_map(timeout_s=10.0)

    def navigate_wait(self, x, y, z, frame_id, speed=SPEED, tolerance=TOL, timeout=WAIT_TIMEOUT, auto_arm=False):
        """Команда + ожидание по телеметрии в том же frame_id."""
        ok = self.navigate(x, y, z, yaw=0.0, speed=speed, auto_arm=auto_arm, frame_id=frame_id)
        if not ok:
            return False

        t0 = time.time()
        while time.time() - t0 < timeout and rclpy.ok():
            telem = self.get_telemetry(frame_id=frame_id)
            if not telem.get("connected"):
                self.sleep_spin(0.2)
                continue

            dx = telem["x"] - x
            dy = telem["y"] - y
            dz = telem["z"] - z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            if dist < tolerance:
                return True

            self.sleep_spin(0.2)

        self.get_logger().warn(f"Timeout waiting target in {frame_id}: x={x} y={y} z={z}")
        return False

    def snap(self, tag: str):
        if not CAMERA_AVAILABLE:
            return False
        if self._last_frame is None:
            self.get_logger().warn("No camera frame yet, skip snap")
            return False

        path = os.path.join(PHOTOS_DIR, f"{tag}.jpg")
        try:
            cv2.imwrite(path, self._last_frame)
            self.get_logger().info(f"📸😂✌️ saved: {path}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to save photo: {e}")
            return False


def telemetry_loop(node: DroneController):
    while rclpy.ok():
        t = node.get_telemetry(frame_id=ARUCO_MAP_FRAME)
        if t.get("connected"):
            node.get_logger().info(
                f"[telem {ARUCO_MAP_FRAME}] x={t['x']:.2f} y={t['y']:.2f} z={t['z']:.2f} "
                f"mode={t.get('mode')} armed={t.get('armed')} V={t.get('voltage'):.2f}"
            )
        else:
            node.get_logger().warn("Telemetry not connected")
        time.sleep(1.0)


def main():
    rclpy.init()
    drone = DroneController()

    # поток телеметрии
    threading.Thread(target=lambda: telemetry_loop(drone), daemon=True).start()

    # === маршрут змейкой ===
    ROUTE = [
        ("aruco_49",  1.6, 0.0, WORK_Z),
        ("aruco_50",  0.8, 0.0, WORK_Z),
        ("aruco_58",  0.0, 0.0, WORK_Z),

        ("aruco_62",  0.0, 0.8, WORK_Z),
        ("CENTER",    0.8, 0.8, CENTER_Z),   
        ("aruco_81",  1.6, 0.8, WORK_Z),

        ("aruco_51",  1.6, 1.6, WORK_Z),
        ("aruco_61",  0.8, 1.6, WORK_Z),
        ("aruco_64",  0.0, 1.6, WORK_Z),
    ]

    try:
        print("\n=== SNAKE (ROS2) ===\n")

        # взлёт
        print(f">>> Takeoff to {TAKEOFF_Z}m (body)")
        if not drone.navigate_wait(0.0, 0.0, TAKEOFF_Z, frame_id=BODY_FRAME, speed=1.0, tolerance=0.25, timeout=25.0, auto_arm=True):
            print("ERROR: Takeoff failed")
            return

        drone.sleep_spin(2.0)

        # ждём aruco_map
        print(">>> Waiting for aruco_map...")
        if not drone.wait_for_aruco_map(timeout_s=12.0):
            print("ERROR: aruco_map not available (no marker localization)")
            return

        # опускаемся на рабочую высоту
        print(f">>> Go to work height {WORK_Z}m (body)")
        drone.navigate_wait(0.0, 0.0, WORK_Z, frame_id=BODY_FRAME, speed=1.0, tolerance=0.20, timeout=15.0, auto_arm=False)
        drone.sleep_spin(1.0)

        # летим змейкой
        for i, (name, x, y, z) in enumerate(ROUTE, start=1):
            print(f">>> [{i}/{len(ROUTE)}] {name}: aruco_map x={x:.2f} y={y:.2f} z={z:.2f}")

            if not drone.ensure_aruco_map():
                print("ERROR: lost aruco_map and can't recover")
                break

            tol = CENTER_TOL if name == "CENTER" else TOL
            ok = drone.navigate_wait(x, y, z, frame_id=ARUCO_MAP_FRAME, speed=SPEED, tolerance=tol, timeout=WAIT_TIMEOUT, auto_arm=False)

            # центр — не зависаем
            drone.sleep_spin(0.3 if name == "CENTER" else 0.8)

            # фото (если есть камера)
            drone.snap(f"{i:02d}_{name}_x{x:.2f}_y{y:.2f}")

            if not ok:
                print(f"WARN: point {name} not reached in time, continuing...")

        print(">>> Landing")
        drone.land()
        time.sleep(5.0)

        print("\n=== DONE ===")

    except KeyboardInterrupt:
        print("\n>>> Interrupted, landing...")
        try:
            drone.land()
            time.sleep(3.0)
        except Exception:
            pass

    finally:
        drone.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()