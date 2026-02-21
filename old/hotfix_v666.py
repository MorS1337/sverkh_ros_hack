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

TAKEOFF_DZ = 0.6      # <-- ВАЖНО: В BODY это ПРИБАВКА к текущей высоте. Поставь 0.5-0.8
SPEED = 0.5

BODY_FRAME = "body"
PHOTOS_DIR = "photos"
os.makedirs(PHOTOS_DIR, exist_ok=True)

CAMERA_SENSOR = "0v5647"
IMAGE_TOPIC = f"/{CAMERA_SENSOR}/image_raw"

# маршрут: 2 вперёд, 1 влево, 2 назад, 1 влево, 2 вперёд
MOVES = [
    ("FWD_2",  +2*CELL, 0.0,     0.0),
    ("LEFT_1", 0.0,     +1*CELL, 0.0),
    ("BACK_2", -2*CELL, 0.0,     0.0),
    ("LEFT_1", 0.0,     +1*CELL, 0.0),
    ("FWD_2",  +2*CELL, 0.0,     0.0),
]

class Drone(Node):
    def __init__(self):
        super().__init__("body_snake")

        self.navigate_client = self.create_client(Navigate, "/navigate")
        self.land_client = self.create_client(Trigger, "/land")
        self.telemetry_client = self.create_client(GetTelemetry, "/get_telemetry")

        self._lock = threading.Lock()
        self._wait_services()

        self.bridge = CvBridge() if CAMERA_AVAILABLE else None
        self._last_frame = None

        if CAMERA_AVAILABLE:
            self.create_subscription(Image, IMAGE_TOPIC, self._img_cb, 10)
            self.get_logger().info(f"Camera ON: {IMAGE_TOPIC}")
        else:
            self.get_logger().warn("Camera OFF (no cv2/cv_bridge).")

        self.get_logger().info("Ready.")

    def _img_cb(self, msg: "Image"):
        try:
            self._last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            pass

    def _wait_services(self):
        for client, name in [
            (self.navigate_client, "Navigate"),
            (self.land_client, "Land"),
            (self.telemetry_client, "Telemetry"),
        ]:
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit("Interrupted")
                self.get_logger().info(f"{name} not available, waiting...")

    def _call(self, client, req, timeout=5.0):
        with self._lock:
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return fut.result() if fut.done() else None

    def navigate(self, x, y, z, speed=SPEED, auto_arm=False, frame_id=BODY_FRAME):
        req = Navigate.Request()
        req.x = float(x)
        req.y = float(y)
        req.z = float(z)
        req.yaw = float("nan")          # <-- чтобы не крутился (обычно это ок)
        req.speed = float(speed)
        req.angular_velocity = 1.0
        req.auto_arm = bool(auto_arm)
        req.frame_id = str(frame_id)

        resp = self._call(self.navigate_client, req, timeout=5.0)
        if resp:
            self.get_logger().info(f'Navigate: success={resp.success} msg="{resp.message}"')
            return resp.success
        self.get_logger().error("Navigate: timeout")
        return False

    def land(self):
        resp = self._call(self.land_client, Trigger.Request(), timeout=5.0)
        return bool(resp and resp.success)

    def sleep_spin(self, sec, step=0.1):
        t0 = time.time()
        while time.time() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=step)

    def snap(self, tag):
        if not CAMERA_AVAILABLE or self._last_frame is None:
            return
        path = os.path.join(PHOTOS_DIR, f"{tag}.jpg")
        try:
            cv2.imwrite(path, self._last_frame)
            self.get_logger().info(f"📸 saved {path}")
        except Exception:
            pass

    def move_body_and_wait(self, name, dx, dy, dz):
        # железно запрещаем набор высоты на шагах
        dz = 0.0

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        wait_s = max(2.0, dist / max(SPEED, 0.01) + 1.5)  # запас

        self.get_logger().info(f"{name}: dx={dx:.2f} dy={dy:.2f} dz={dz:.2f} wait≈{wait_s:.1f}s")
        if not self.navigate(dx, dy, dz, speed=SPEED, auto_arm=False, frame_id=BODY_FRAME):
            self.get_logger().warn(f"{name}: navigate failed")
            return

        self.sleep_spin(wait_s)

        # HOLD (убивает “интеграцию”, если она есть)
        self.navigate(0.0, 0.0, 0.0, speed=0.3, auto_arm=False, frame_id=BODY_FRAME)
        self.sleep_spin(0.4)

        self.snap(name)

def main():
    rclpy.init()
    d = Drone()

    try:
        print("\n=== BODY SNAKE ===\n")

        # Взлёт: В BODY z — это прибавка, поэтому делаем ОДИН раз
        print(f">>> Takeoff +{TAKEOFF_DZ}m (body)")
        if not d.navigate(0.0, 0.0, TAKEOFF_DZ, speed=0.8, auto_arm=True, frame_id=BODY_FRAME):
            print("Takeoff failed")
            return

        d.sleep_spin(max(4.0, TAKEOFF_DZ/0.8 + 3.0))
        d.navigate(0.0, 0.0, 0.0, speed=0.3, auto_arm=False, frame_id=BODY_FRAME)  # hold
        d.sleep_spin(0.5)
        d.snap("00_TAKEOFF")

        # Змейка
        for i, (name, dx, dy, dz) in enumerate(MOVES, start=1):
            d.move_body_and_wait(f"{i:02d}_{name}", dx, dy, dz)

        print(">>> Landing")
        d.land()
        time.sleep(3)

    except KeyboardInterrupt:
        print("Interrupted -> landing")
        try:
            d.land()
        except Exception:
            pass
    finally:
        d.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()