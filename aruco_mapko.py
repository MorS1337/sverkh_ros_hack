#!/usr/bin/env python3
import os, math, time, threading, glob, sys

import rclpy
from rclpy.node import Node

from offboard_interfaces.srv import Navigate, GetTelemetry
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

# ========= CONFIG =========
ARUCO_MAP_FRAME = "aruco_map"
IMAGE_TOPIC = "/aruco_det/debug_image"  # оставляем как просил

# маршрут по айди (как ты написал)
ROUTE = [49, 81, 61, "CENTER", 50, 58, 62, 64]

TAKEOFF_Z = 1.2     # взлёт (в body, абсолют в "body" = прибавка к текущей высоте)
WORK_Z = 0.8        # высота на клетках
CENTER_Z = 1.1      # центр чуть выше
SPEED = 0.55
ANG_VEL = 1.0

TOL = 0.20
CENTER_TOL = 0.30
WAIT_TIMEOUT = 20.0
HOVER_S = 0.6

PHOTOS_DIR = "photos"
RESULTS_DIR = "results"
os.makedirs(PHOTOS_DIR, exist_ok=True)
os.makedirs(RESULTS_DIR, exist_ok=True)

# YOLO после посадки (можно отключить)
RUN_YOLO_AFTER_LAND = True
YOLO_AVAILABLE = True
try:
    from ultralytics import YOLO
except Exception:
    YOLO_AVAILABLE = False
YOLO_MODEL_PATH = "./yolov8n_ncnn_model"
YOLO_CONF = 0.45
TARGET_CLASSES = {"orange", "teddy bear"}

# ==========================

def finite(x: float) -> bool:
    return math.isfinite(x) and not math.isnan(x)

class FinalArucoMapCanon(Node):
    def __init__(self):
        super().__init__("final_aruco_map_canon")

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # camera
        self.bridge = CvBridge()
        self._last_frame = None
        self.create_subscription(Image, IMAGE_TOPIC, self._img_cb, 10)

        # services
        self.nav_cli = self.create_client(Navigate, "/navigate")
        self.tel_cli = self.create_client(GetTelemetry, "/get_telemetry")
        self.land_cli = self.create_client(Trigger, "/land")

        self._lock = threading.Lock()
        self._wait_services()

        self.marker_cache = {}  # mid -> (x,y)
        self._navigate_target_supported = None  # определим на лету

        self.get_logger().info("READY")

    def _img_cb(self, msg: Image):
        try:
            self._last_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception:
            pass

    def sleep_spin(self, sec: float, step=0.1):
        t0 = time.time()
        while time.time() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=step)

    def _wait_services(self):
        for cli, name in [(self.nav_cli, "navigate"), (self.tel_cli, "get_telemetry"), (self.land_cli, "land")]:
            while not cli.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit("ROS2 interrupted")
                self.get_logger().info(f"Waiting for /{name}...")

    def _call(self, cli, req, timeout=5.0):
        with self._lock:
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
            return fut.result() if fut.done() else None

    # ---------- services ----------
    def get_telemetry(self, frame_id=None):
        req = GetTelemetry.Request()
        if frame_id is not None and hasattr(req, "frame_id"):
            req.frame_id = str(frame_id)
        resp = self._call(self.tel_cli, req, timeout=3.0)
        if resp and getattr(resp, "connected", False):
            return resp
        return None

    def navigate(self, x, y, z, frame_id, auto_arm=False, speed=SPEED, yaw=float("nan")):
        req = Navigate.Request()
        req.x = float(x); req.y = float(y); req.z = float(z)
        req.yaw = float(yaw)
        req.speed = float(speed)
        if hasattr(req, "angular_velocity"):
            req.angular_velocity = float(ANG_VEL)
        req.frame_id = str(frame_id)
        req.auto_arm = bool(auto_arm)

        resp = self._call(self.nav_cli, req, timeout=5.0)
        if not resp:
            self.get_logger().error("Navigate timeout")
            return False
        self.get_logger().info(f'Navigate: success={resp.success} msg="{resp.message}"')
        return bool(resp.success)

    def land(self):
        resp = self._call(self.land_cli, Trigger.Request(), timeout=5.0)
        if resp:
            self.get_logger().info(f'Land: success={resp.success} msg="{resp.message}"')
            return bool(resp.success)
        self.get_logger().error("Land timeout")
        return False

    # ---------- clover-canon: navigate_wait via navigate_target ----------
    def _detect_navigate_target_support(self) -> bool:
        """Пробуем один раз понять, работает ли frame_id='navigate_target'."""
        t = self.get_telemetry(frame_id="navigate_target")
        ok = bool(t and finite(t.x) and finite(t.y) and finite(t.z))
        self._navigate_target_supported = ok
        self.get_logger().info(f"navigate_target support: {ok}")
        return ok

    def navigate_wait(self, x, y, z, frame_id, tolerance=TOL, timeout=WAIT_TIMEOUT, auto_arm=False):
        """
        Канон Clover:
        1) отправили navigate
        2) ждём, пока get_telemetry('navigate_target') станет < tolerance
        Fallback: если navigate_target не поддержан — считаем расстояние в frame_id до (x,y,z).
        """
        if not self.navigate(x, y, z, frame_id=frame_id, auto_arm=auto_arm, speed=SPEED, yaw=float("nan")):
            return False

        if self._navigate_target_supported is None:
            self._detect_navigate_target_support()

        t0 = time.time()
        while time.time() - t0 < timeout and rclpy.ok():
            if self._navigate_target_supported:
                t = self.get_telemetry(frame_id="navigate_target")
                if t and finite(t.x) and finite(t.y) and finite(t.z):
                    dist = math.sqrt(t.x*t.x + t.y*t.y + t.z*t.z)
                    if dist < tolerance:
                        return True
                self.sleep_spin(0.2)
                continue

            # fallback: сравниваем текущую позицию в том же frame_id
            t = self.get_telemetry(frame_id=frame_id)
            if t and finite(t.x) and finite(t.y) and finite(t.z):
                dx = t.x - x; dy = t.y - y; dz = t.z - z
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                if dist < tolerance:
                    return True
            self.sleep_spin(0.2)

        self.get_logger().warn(f"Timeout waiting target ({frame_id}) x={x:.2f} y={y:.2f} z={z:.2f}")
        return False

    # ---------- aruco_map availability / recovery ----------
    def wait_for_aruco_map(self, timeout_s=15.0) -> bool:
        t0 = time.time()
        while time.time() - t0 < timeout_s and rclpy.ok():
            t = self.get_telemetry(frame_id=ARUCO_MAP_FRAME)
            if t and finite(t.x) and finite(t.y) and finite(t.z):
                return True
            self.sleep_spin(0.2)
        return False

    def ensure_aruco_map(self) -> bool:
        """Если aruco_map отвалился — поднимемся в body и попробуем восстановить."""
        if self.wait_for_aruco_map(timeout_s=1.5):
            return True
        self.get_logger().warn("aruco_map lost -> ascend for recovery...")
        self.navigate(0.0, 0.0, 0.6, frame_id="body", auto_arm=False, speed=0.8, yaw=float("nan"))
        self.sleep_spin(1.0)
        return self.wait_for_aruco_map(timeout_s=12.0)

    # ---------- TF marker coords ----------
    def marker_xy(self, mid: int):
        frame = f"aruco_{mid}"
        try:
            tr = self.tf_buffer.lookup_transform(ARUCO_MAP_FRAME, frame, rclpy.time.Time())
            x = tr.transform.translation.x
            y = tr.transform.translation.y
            if finite(x) and finite(y):
                self.marker_cache[mid] = (x, y)
                return x, y
        except Exception:
            pass

        if mid in self.marker_cache:
            self.get_logger().warn(f"No TF for {frame}, using cached coords")
            return self.marker_cache[mid]

        return None

    def compute_center_xy(self):
        # best: diagonal midpoint if possible
        pairs = [(49, 64), (58, 61), (81, 62), (50, 61)]
        for a, b in pairs:
            pa = self.marker_xy(a)
            pb = self.marker_xy(b)
            if pa and pb:
                return ((pa[0] + pb[0]) / 2.0, (pa[1] + pb[1]) / 2.0)

        if self.marker_cache:
            xs = [p[0] for p in self.marker_cache.values()]
            ys = [p[1] for p in self.marker_cache.values()]
            return (sum(xs) / len(xs), sum(ys) / len(ys))

        # absolute fallback
        return (0.8, 0.8)

    # ---------- photo ----------
    def snap(self, tag: str):
        if self._last_frame is None:
            self.get_logger().warn("No frame yet, skip photo")
            return False
        path = os.path.join(PHOTOS_DIR, f"{tag}.jpg")
        try:
            cv2.imwrite(path, self._last_frame)
            self.get_logger().info(f"📸 saved: {path}")
            return True
        except Exception as e:
            self.get_logger().warn(f"save photo failed: {e}")
            return False

    # ---------- YOLO postflight ----------
    def yolo_after(self):
        if not RUN_YOLO_AFTER_LAND:
            self.get_logger().info("YOLO after landing: disabled")
            return
        if not YOLO_AVAILABLE:
            self.get_logger().warn("YOLO after landing: ultralytics not installed")
            return
        if not os.path.exists(YOLO_MODEL_PATH):
            self.get_logger().warn(f"YOLO model not found: {YOLO_MODEL_PATH}")
            return

        photos = sorted(glob.glob(os.path.join(PHOTOS_DIR, "cell_*.jpg")))
        if not photos:
            self.get_logger().warn("No photos cell_*.jpg to run YOLO on")
            return

        self.get_logger().info("Loading YOLO (postflight)...")
        model = YOLO(YOLO_MODEL_PATH)

        for p in photos:
            try:
                results = model.predict(source=p, conf=YOLO_CONF, verbose=False)
            except Exception as e:
                self.get_logger().warn(f"YOLO failed on {p}: {e}")
                continue

            found = set()
            for r in results:
                for box in getattr(r, "boxes", []):
                    cls_id = int(box.cls[0])
                    name = model.names.get(cls_id, str(cls_id))
                    if name in TARGET_CLASSES:
                        found.add(name)

            # always save annotated for judge folder browsing
            try:
                annotated = results[0].plot() if results else None
                if annotated is not None:
                    out = os.path.join(RESULTS_DIR, "DETECTED_" + os.path.basename(p))
                    cv2.imwrite(out, annotated)
            except Exception:
                pass

            if found:
                self.get_logger().info(f"✅ {os.path.basename(p)} -> {sorted(found)}")
            else:
                self.get_logger().info(f"— {os.path.basename(p)} -> empty")

        self.get_logger().info(f"Proofs: {RESULTS_DIR}/DETECTED_cell_*.jpg")


def main():
    rclpy.init()
    node = FinalArucoMapCanon()

    try:
        print("\n=== FINAL CANON: ARUCO_MAP + NAVIGATE_TARGET WAIT ===\n")

        # takeoff in body (only once auto_arm=True)
        node.get_logger().info("Takeoff (body) auto_arm=True")
        if not node.navigate(0.0, 0.0, TAKEOFF_Z, frame_id="body", auto_arm=True, speed=0.9, yaw=float("nan")):
            node.get_logger().error("Takeoff failed")
            return
        node.sleep_spin(3.0)

        # wait for aruco_map
        node.get_logger().info("Waiting aruco_map...")
        if not node.wait_for_aruco_map(timeout_s=18.0):
            node.get_logger().error("aruco_map not available -> landing")
            node.land()
            return

        # fly route strictly in aruco_map
        for i, step in enumerate(ROUTE, start=1):
            tag = f"cell_{i:02d}"

            if not node.ensure_aruco_map():
                node.get_logger().error("Lost aruco_map and can't recover -> landing")
                node.land()
                return

            if step == "CENTER":
                cx, cy = node.compute_center_xy()
                node.get_logger().info(f"[{i}/{len(ROUTE)}] CENTER -> ({cx:.2f},{cy:.2f},{CENTER_Z:.2f}) aruco_map")
                node.navigate_wait(cx, cy, CENTER_Z, frame_id=ARUCO_MAP_FRAME,
                                  tolerance=CENTER_TOL, timeout=WAIT_TIMEOUT, auto_arm=False)
                node.sleep_spin(HOVER_S)
                node.snap(tag)
                continue

            mid = int(step)
            xy = node.marker_xy(mid)
            if not xy:
                node.get_logger().error(f"No TF for aruco_{mid} (aruco_map->aruco_{mid}) -> landing")
                node.land()
                return

            x, y = xy
            node.get_logger().info(f"[{i}/{len(ROUTE)}] aruco_{mid} -> ({x:.2f},{y:.2f},{WORK_Z:.2f}) aruco_map")
            node.navigate_wait(x, y, WORK_Z, frame_id=ARUCO_MAP_FRAME,
                              tolerance=TOL, timeout=WAIT_TIMEOUT, auto_arm=False)
            node.sleep_spin(HOVER_S)
            node.snap(tag)

        # land
        node.get_logger().info("Mission complete -> landing")
        node.land()
        node.sleep_spin(3.0)

        # postflight YOLO
        node.yolo_after()

        node.get_logger().info("DONE ✅")

    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted -> landing")
        try:
            node.land()
        except Exception:
            pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
    
