#!/usr/bin/env python3
import os, math, time, threading, sys
import rclpy
from rclpy.node import Node

from offboard_interfaces.srv import Navigate, GetTelemetry
from std_srvs.srv import Trigger

# OPTIONAL services (if available in your build)
SET_ALT_AVAILABLE = True
try:
    from offboard_interfaces.srv import SetAltitude
except Exception:
    SET_ALT_AVAILABLE = False

# camera optional
CAMERA_AVAILABLE = True
try:
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
    import cv2
except Exception:
    CAMERA_AVAILABLE = False

# ===== CONFIG =====
CELL = 0.8
SPEED = 0.4                 # помедленнее = стабильнее
TAKEOFF_DZ = 0.6            # В BODY это ПРИБАВКА. делай 0.5-0.8
HOLD_ALT = 0.8              # какую высоту держим (лучше terrain)
ALT_FRAME = "terrain"       # "terrain" если работает, иначе "map"
ALT_RATE = 0.4              # как часто поджимать высоту (сек)

BODY_FRAME = "body"

PHOTOS_DIR = "photos"
os.makedirs(PHOTOS_DIR, exist_ok=True)
CAMERA_SENSOR = "0v5647"
IMAGE_TOPIC = f"/{CAMERA_SENSOR}/image_raw"

MOVES = [
    ("CELL_01_START",  0.0,    0.0,   0.0),   

    ("CELL_02_FWD",   +0.8,    0.0,   0.0),
    ("CELL_03_FWD",   +0.8,    0.0,   0.0),

    ("CELL_04_LEFT",   0.0,   +0.8,   0.0),

    ("CELL_05_BACK",  -0.8,    0.0,   0.0),
    ("CELL_06_BACK",  -0.8,    0.0,   0.0),

    ("CELL_07_LEFT",   0.0,   +0.8,   0.0),

    ("CELL_08_FWD",   +0.8,    0.0,   0.0),
    ("CELL_09_FWD",   +0.8,    0.0,   0.0),
]

class Drone(Node):
    def __init__(self):
        super().__init__("snake_body_mission")

        self.navigate_client = self.create_client(Navigate, "/navigate")
        self.land_client = self.create_client(Trigger, "/land")
        self.telemetry_client = self.create_client(GetTelemetry, "/get_telemetry")

        self.set_alt_client = None
        if SET_ALT_AVAILABLE:
            self.set_alt_client = self.create_client(SetAltitude, "/set_altitude")

        self._lock = threading.Lock()
        self._wait_services()

        self.bridge = CvBridge() if CAMERA_AVAILABLE else None
        self._last_frame = None
        if CAMERA_AVAILABLE:
            self.create_subscription(Image, IMAGE_TOPIC, self._img_cb, 10)
            self.get_logger().info(f"Camera ON: {IMAGE_TOPIC}")
        else:
            self.get_logger().warn("Camera OFF (no cv2/cv_bridge).")

        self._alt_hold_on = False
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

        if self.set_alt_client is not None:
            t0 = time.time()
            while not self.set_alt_client.wait_for_service(timeout_sec=1.0):
                if time.time() - t0 > 3.0:
                    self.get_logger().warn("/set_altitude not available -> altitude hold thread disabled")
                    self.set_alt_client = None
                    break
                self.get_logger().info("SetAltitude not available, waiting...")

    def _call(self, client, req, timeout=5.0):
        with self._lock:
            fut = client.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return fut.result() if fut.done() else None

    def navigate(self, x, y, z, speed=SPEED, auto_arm=False, frame_id=BODY_FRAME):
        req = Navigate.Request()
        req.x = float(x); req.y = float(y); req.z = float(z)
        req.yaw = 0.0
        req.speed = float(speed)
        req.angular_velocity = 1.0
        req.auto_arm = bool(auto_arm)
        req.frame_id = str(frame_id)

        resp = self._call(self.navigate_client, req, timeout=5.0)
        if resp:
            self.get_logger().info(f'Navigate: success={resp.success} msg="{resp.message}"')
            return resp.success
        self.get_logger().error("Navigate timeout")
        return False

    def set_altitude(self, z, frame_id=ALT_FRAME):
        if self.set_alt_client is None:
            return False
        req = SetAltitude.Request()
        req.z = float(z)
        req.frame_id = str(frame_id)
        resp = self._call(self.set_alt_client, req, timeout=3.0)
        return bool(resp and resp.success)

    def land(self):
        resp = self._call(self.land_client, Trigger.Request(), timeout=5.0)
        if resp:
            self.get_logger().info(f'Land: success={resp.success} msg="{resp.message}"')
        return bool(resp and resp.success)

    def get_telemetry(self, frame_id=None):
        req = GetTelemetry.Request()
        if frame_id is not None and hasattr(req, "frame_id"):
            req.frame_id = str(frame_id)
        resp = self._call(self.telemetry_client, req, timeout=3.0)
        return resp if (resp and getattr(resp, "connected", False)) else None

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
            self.get_logger().info(f"📸 saved: {path}")
        except Exception:
            pass

    def start_alt_hold(self):
        if self.set_alt_client is None:
            self.get_logger().warn("Altitude hold disabled (no /set_altitude)")
            return
        self._alt_hold_on = True

        def loop():
            while rclpy.ok() and self._alt_hold_on:
                try:
                    self.set_altitude(HOLD_ALT, frame_id=ALT_FRAME)
                except Exception:
                    pass
                time.sleep(ALT_RATE)

        threading.Thread(target=loop, daemon=True).start()
        self.get_logger().info(f"Altitude hold ON: z={HOLD_ALT} frame={ALT_FRAME} period={ALT_RATE}s")

    def stop_alt_hold(self):
        self._alt_hold_on = False

    def move_body_and_wait(self, name, dx, dy, dz):
        dz = 0.0

        if abs(dx) < 1e-6 and abs(dy) < 1e-6 and abs(dz) < 1e-6:
            self.navigate(0.0, 0.0, 0.0, speed=0.3, auto_arm=False, frame_id=BODY_FRAME)  # hold
            self.sleep_spin(0.6)
            self.snap(name)
            return True

        dist = math.sqrt(dx*dx + dy*dy)
        wait_s = max(2.0, dist / max(SPEED, 0.01) + 1.5)

        self.get_logger().info(f"{name}: dx={dx:.2f} dy={dy:.2f} dz=0.00 wait≈{wait_s:.1f}s")
        ok = self.navigate(dx, dy, 0.0, speed=SPEED, auto_arm=False, frame_id=BODY_FRAME)
        if not ok:
            return False

        self.sleep_spin(wait_s)

        self.navigate(0.0, 0.0, 0.0, speed=0.3, auto_arm=False, frame_id=BODY_FRAME)
        self.sleep_spin(0.4)

        self.snap(name)
        return True

def telemetry_loop(node: Drone):
    while rclpy.ok():
        t_map = node.get_telemetry(frame_id="map")
        t_alt = node.get_telemetry(frame_id=ALT_FRAME) if ALT_FRAME != "map" else None

        if t_map:
            z_show = t_alt.z if t_alt else t_map.z
            node.get_logger().info(
                f"[telem] mode={getattr(t_map,'mode','?')} armed={getattr(t_map,'armed',False)} "
                f"x={t_map.x:.2f} y={t_map.y:.2f} z={z_show:.2f}V={getattr(t_map,'voltage',float('nan')):.2f}"
            )
        else:
            node.get_logger().warn("Telemetry not connected")
        time.sleep(1.0)

def main():
    rclpy.init()
    d = Drone()
    threading.Thread(target=lambda: telemetry_loop(d), daemon=True).start()

    try:
        print("\n=== BODY SNAKE + ALT HOLD ===\n")

        print(f">>> Takeoff +{TAKEOFF_DZ}m (body)")
        if not d.navigate(0.0, 0.0, TAKEOFF_DZ, speed=0.8, auto_arm=True, frame_id=BODY_FRAME):
            print("Takeoff failed"); return

        d.sleep_spin(4.0)
        d.navigate(0.0, 0.0, 0.0, speed=0.3, auto_arm=False, frame_id=BODY_FRAME)
        d.sleep_spin(0.5)

        d.start_alt_hold()

        d.snap("cell_01") 

        # змейка
        for i, (_name, dx, dy, dz) in enumerate(MOVES, start=1):
            d.move_body_and_wait(f"cell_{i:02d}", dx, dy, dz)

        print(">>> Landing")
        d.stop_alt_hold()
        d.land()
        time.sleep(3)

    except KeyboardInterrupt:
        print("Interrupted -> landing")
        try:
            d.stop_alt_hold()
            d.land()
        except Exception:
            pass
    finally:
        d.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()