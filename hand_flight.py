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
TAKEOFF_Z = 0.8       # взлет на 0.8 (или сколько вам надо)
SPEED = 0.6
ANG_VEL = 1.0

BODY_FRAME = "body"

PHOTOS_DIR = "photos"
os.makedirs(PHOTOS_DIR, exist_ok=True)

IMAGE_TOPIC = "/main_camera/image_raw" # aruco_det/debug_image чтобы метки было видно

# 2 клетки вперёд, 1 влево, 2 назад, 1 влево, 2 вперёд
BODY_MOVES = [
    ("FWD_2",   +2*CELL, 0.0,     0.0),
    ("LEFT_1",  0.0,     +1*CELL, 0.0),
    ("BACK_2",  -2*CELL, 0.0,     0.0),
    ("LEFT_1",  0.0,     +1*CELL, 0.0),
    ("FWD_2",   +2*CELL, 0.0,     0.0),
]

class DroneController(Node):
    def __init__(self):
        super().__init__("snake_body_mission")

        self.navigate_client = self.create_client(Navigate, "/navigate")
        self.land_client = self.create_client(Trigger, "/land")
        self.telemetry_client = self.create_client(GetTelemetry, "/get_telemetry")

        self._lock = threading.Lock()
        self.wait_for_services()

        # camera
        self.bridge = CvBridge() if CAMERA_AVAILABLE else None
        self._last_frame = None

        if CAMERA_AVAILABLE:
            self.create_subscription(Image, IMAGE_TOPIC, self._image_cb, 10)
            self.get_logger().info(f"Camera subscription ON: {IMAGE_TOPIC}")
        else:
            self.get_logger().warn("Camera disabled (no cv_bridge/sensor_msgs/cv2).")

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

    def snap(self, tag: str):
        if not CAMERA_AVAILABLE:
            return False
        if self._last_frame is None:
            self.get_logger().warn("No camera frame yet, skip snap")
            return False
        path = os.path.join(PHOTOS_DIR, f"{tag}.jpg")
        try:
            cv2.imwrite(path, self._last_frame)
            self.get_logger().info(f"📸 saved: {path}")
            return True
        except Exception as e:
            self.get_logger().warn(f"Failed to save photo: {e}")
            return False

    def move_body_and_wait(self, dx, dy, dz, name, speed=SPEED):
        """Команда в body + ожидание по времени (дистанция/скорость + запас)."""
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        wait_s = max(3.0, dist / max(speed, 0.01) + 3.0)  # +3с запас

        self.get_logger().info(f"Move {name}: body dx={dx:.2f} dy={dy:.2f} dz={dz:.2f} | wait≈{wait_s:.1f}s")
        ok = self.navigate(dx, dy, dz, yaw=0.0, speed=speed, auto_arm=False, frame_id=BODY_FRAME)
        if not ok:
            self.get_logger().warn(f"Move {name}: navigate failed")
            return False

        self.sleep_spin(wait_s)
        self.snap(name)
        return True


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

        # Подождём подольше (взлет)
        drone.sleep_spin(max(5.0, TAKEOFF_Z/1.0 + 3.0))
        drone.snap("00_TAKEOFF")


        # Полёт по змейке (relative body)
        for i, (name, dx, dy, dz) in enumerate(BODY_MOVES, start=1):
            drone.move_body_and_wait(dx, dy, dz, f"{i:02d}_{name}", speed=SPEED)

        print(">>> Landing")
        drone.land()
        time.sleep(4.0)

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