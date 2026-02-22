#!/usr/bin/env python3
import os
import math
import time
import threading
import sys

import rclpy
from rclpy.node import Node
from recognize import RecognizeImage

from offboard_interfaces.srv import Navigate, GetTelemetry
from std_srvs.srv import Trigger

# ===== камера (опционально) =====
CAMERA_AVAILABLE = True
try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
except Exception:
    CAMERA_AVAILABLE = False

# ===== конфиг =====
CELL = 0.8
CELL_POGRESH = 0.15
TAKEOFF_Z = 0.4
SPEED = 0.5
ANG_VEL = 1.0

BODY_FRAME = "body"

PHOTOS_DIR = "photos"
os.makedirs(PHOTOS_DIR, exist_ok=True)

CAMERA_SENSOR = "main_camera"
IMAGE_TOPIC = "/aruco_det/debug_image"

# Маршрут “как ты описал”
# 2 клетки вперёд, 1 влево, 2 назад, 1 влево, 2 вперёд
BODY_MOVES = [
    ("FWD_2",   +2*(CELL-CELL_POGRESH-0.05), 0.0,     0.0),
    ("LEFT_1",  0.0,     +1*CELL, 0.0),
    ("BACK_2",  -2*(CELL-CELL_POGRESH), 0.0,     0.0),
    ("LEFT_1",  0.0,     +1*CELL, 0.0),
    ("FWD_2",   +2*CELL, 0.0,     0.0),
]

class DroneController(Node):
    def __init__(self):
        super().__init__("snake_body_mission")
        self._shooting = False  # флаг для фотопотока

        self.navigate_client = self.create_client(Navigate, "/navigate")
        self.land_client = self.create_client(Trigger, "/land")
        self.telemetry_client = self.create_client(GetTelemetry, "/get_telemetry")

        self._lock = threading.Lock()
        self.wait_for_services()

        # camera
        self.bridge = CvBridge() if CAMERA_AVAILABLE else None
        self._last_frame = None

        self.create_subscription(Image, IMAGE_TOPIC, self._image_cb, 10)
        self.get_logger().info(f"Camera subscription ON: {IMAGE_TOPIC}")

        self.get_logger().info("All services are ready")

    def _image_cb(self, msg: "Image"):
        try:
            self._last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def wait_for_services(self):
        services = [
            (self.navigate_client, "Navigate"),
            (self.land_client, "Land"),
            (self.telemetry_client, "Telemetry"),
        ]
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit("ROS2 interrupted, exiting...")
                self.get_logger().info(f"{name} service not available, waiting...")

    def call_service(self, client, request, timeout_sec=5.0):
        with self._lock:
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
            if not future.done():
                self.get_logger().error("Service call timed out")
                return None
            try:
                return future.result()
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")
                return None

    def navigate(self, x, y, z, yaw=0.0, speed=SPEED, auto_arm=False, frame_id=BODY_FRAME):
        req = Navigate.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float(yaw)
        req.speed = float(speed)
        req.angular_velocity = float(ANG_VEL)
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

    def get_telemetry(self):
        req = GetTelemetry.Request()
        resp = self.call_service(self.telemetry_client, req, timeout_sec=3.0)
        if resp and getattr(resp, "connected", False):
            return resp
        return None

    def sleep_spin(self, duration_s: float, step=0.1):
        t0 = time.time()
        while time.time() - t0 < duration_s and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=step)

    def shot(self):
        if self._last_frame is None:
            self.get_logger().warn("No camera frame yet, skip snap")
            return False
        
        timestamp = int(time.time())
        path = os.path.join("photos", f"shot_{timestamp}.jpg")
        try:
            cv2.imwrite(path, self._last_frame)
            self.get_logger().info(f"📸 Saved: {path}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to save photo: {e}")
            return False

    def move_body_and_wait(self, dx, dy, dz, speed=SPEED):
        """Команда в body + ожидание по времени (дистанция/скорость + запас)."""
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        wait_s = max(2.0, dist / max(speed, 0.01) + 2.0)  # +2с запас

        self.get_logger().info(f"Move: body dx={dx:.2f} dy={dy:.2f} dz={dz:.2f} | wait≈{wait_s:.1f}s")
        ok = self.navigate(dx, dy, dz, yaw=0.0, speed=speed, auto_arm=False, frame_id=BODY_FRAME)
        if not ok:
            self.get_logger().warn("Move: navigate failed")
            return False
       # self.shot()
        self.sleep_spin(wait_s)
        #self.shot()
        return True
    
    def start_shooting(self, interval=1.0):
        """Запускает фоновый поток, который фотографирует каждые interval секунд."""
        self._shooting = True
        def _loop():
            while self._shooting and rclpy.ok():
                self.shot()
                time.sleep(interval)
        self._shoot_thread = threading.Thread(target=_loop, daemon=True)
        self._shoot_thread.start()
        self.get_logger().info(f"📸 Auto-shooting started (every {interval}s)")

    def stop_shooting(self):
        """Останавливает фоновый поток фотографирования."""
        self._shooting = False
        self.get_logger().info("📸 Auto-shooting stopped")



def telemetry_loop(node: DroneController):
    while rclpy.ok():
        t = node.get_telemetry()
        if t:
            node.get_logger().info(
                f"[telem] mode={getattr(t,'mode','?')} armed={getattr(t,'armed',False)} "
                f"x={t.x:.2f} y={t.y:.2f} z={t.z:.2f} V={getattr(t,'voltage',float('nan')):.2f}"
            )
        else:
            node.get_logger().warn("Telemetry not connected")
        time.sleep(1.0)


def main():
    rclpy.init()
    drone = DroneController()

    threading.Thread(target=lambda: telemetry_loop(drone), daemon=True).start()

    try:
        print("\n=== BODY SNAKE (ROS2) ===\n")

        # Взлет (relative body)
        print(f">>> Takeoff to {TAKEOFF_Z}m (body)")
        ok = drone.navigate(0.0, 0.0, TAKEOFF_Z, yaw=0.0, speed=1.0, auto_arm=True, frame_id=BODY_FRAME)
        if not ok:
            print("ERROR: Takeoff failed")
            return

        drone.start_shooting(interval=1.0)

        # Подождём подольше (взлет)
        drone.sleep_spin(max(5.0, TAKEOFF_Z/1.0 + 3.0))

        # Полёт по змейке (relative body)
        for i, (name, dx, dy, dz) in enumerate(BODY_MOVES, start=1):
            drone.move_body_and_wait(dx, dy, dz, speed=SPEED)
            t = drone.get_telemetry()
            if t.z > (TAKEOFF_Z+0.3): 
                drone.move_body_and_wait(0, 0, -abs((TAKEOFF_Z+0.3)-t), speed=SPEED)

        drone.stop_shooting()
        
        print(">>> Landing")
        drone.land()
        time.sleep(4.0)
        rec = RecognizeImage()
        rec.start()

        print("\n=== DONE ===")

    except KeyboardInterrupt:
        print("\n>>> Interrupted, landing...")
        try:
            drone.land()
            time.sleep(4.0)
            rec = RecognizeImage()
            rec.start()
        except Exception:
            drone.land()
            time.sleep(4.0)
            rec = RecognizeImage()
            rec.start()
            pass
        
    except Exception:
            drone.land()
            time.sleep(4.0)
            rec = RecognizeImage()
            rec.start()
            pass

    finally:
        drone.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()