#!/usr/bin/env python3
import os
import math
import time
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover import srv
from std_srvs.srv import Trigger
from clover.srv import SetLEDEffect

# ====== НАСТРОЙКИ ПОЛЯ ======
CELL = 0.8          # сторона клетки (м)
TAKEOFF_Z = 1.5     # чтобы уверенно увидеть маркеры и поднять aruco_map
WORK_Z = 0.8        # рабочая высота над клетками
CENTER_Z = 1.4      # центр (без метки) выше -> лучше видны соседние маркеры

PHOTOS_DIR = "photos"
os.makedirs(PHOTOS_DIR, exist_ok=True)

rospy.init_node("flight")
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy("get_telemetry", srv.GetTelemetry)
navigate = rospy.ServiceProxy("navigate", srv.Navigate)
land = rospy.ServiceProxy("land", Trigger)
set_effect = rospy.ServiceProxy("led/set_effect", SetLEDEffect)

def navigate_wait(x=0, y=0, z=0, yaw=float("nan"), speed=0.5, frame_id="body",
                  auto_arm=False, tolerance=0.2):
    navigate(x=x, y=y, z=z, yaw=yaw, speed=speed, frame_id=frame_id, auto_arm=auto_arm)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id="navigate_target")
        dist = math.sqrt(telem.x**2 + telem.y**2 + telem.z**2)
        if dist < tolerance:
            return True
        rospy.sleep(0.2)
    return False

def wait_for_aruco_map(timeout_s=10.0):
    t0 = time.time()
    while time.time() - t0 < timeout_s and not rospy.is_shutdown():
        try:
            telem = get_telemetry(frame_id="aruco_map")
            if math.isfinite(telem.x) and math.isfinite(telem.y):
                return True
        except Exception:
            pass
        rospy.sleep(0.2)
    return False

def ensure_aruco_map():
    """Если aruco_map отвалился — поднимаемся и ждём, пока снова появится."""
    if wait_for_aruco_map(timeout_s=1.5):
        return True
    rospy.logwarn("aruco_map пропал -> поднимаюсь и пытаюсь восстановить локализацию")
    set_effect(effect="blink_fast", r=255, g=80, b=0)
    navigate_wait(z=TAKEOFF_Z, frame_id="body", tolerance=0.25)
    rospy.sleep(1.0)
    ok = wait_for_aruco_map(timeout_s=8.0)
    if not ok:
        set_effect(effect="blink_fast", r=255, g=0, b=0)
    return ok

def snap(tag: str, timeout=5.0):
    try:
        msg = rospy.wait_for_message("main_camera/image_raw", Image, timeout=timeout)
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")
        path = os.path.join(PHOTOS_DIR, f"{tag}.jpg")
        cv2.imwrite(path, frame)
        rospy.loginfo(f"📸 saved: {path}")
        set_effect(effect="flash", r=255, g=255, b=0)  
        return True
    except rospy.ROSException:
        rospy.logwarn("❌ camera timeout, skip")
        set_effect(effect="blink_fast", r=255, g=0, b=0)
        return False

# ====== ТВОЯ МАТРИЦА (снизу вверх по Y) ======
# y=0.0: [49, 81, 51]
# y=0.8: [50, None, 61]
# y=1.6: [58, 62, 64]
# Змейка начиная с 49:
ROUTE = [
    (0*CELL, 0*CELL, 49),
    (1*CELL, 0*CELL, 81),
    (2*CELL, 0*CELL, 51),

    (2*CELL, 1*CELL, 61),
    (1*CELL, 1*CELL, None),   
    (0*CELL, 1*CELL, 50),

    (0*CELL, 2*CELL, 58),
    (1*CELL, 2*CELL, 62),
    (2*CELL, 2*CELL, 64),
]

try:
    set_effect(r=0, g=0, b=255)  
    rospy.sleep(0.5)

    navigate_wait(z=TAKEOFF_Z, frame_id="body", auto_arm=True, tolerance=0.2)
    rospy.sleep(2.0)

    if not wait_for_aruco_map(timeout_s=10.0):
        set_effect(effect="blink_fast", r=255, g=0, b=0)
        raise RuntimeError("aruco_map не появился (дрон не видит карту)")

    navigate_wait(z=WORK_Z, frame_id="body", tolerance=0.15)
    rospy.sleep(1.0)

    for i, (x, y, mid) in enumerate(ROUTE, start=1):
        if not ensure_aruco_map():
            raise RuntimeError("Потеря локализации: aruco_map не восстановился")

        is_center = (mid is None)
        z = CENTER_Z if is_center else WORK_Z
        tol = 0.25 if is_center else 0.18

        target_name = "CENTER" if is_center else f"aruco_{mid}"
        rospy.loginfo(f"[{i}/{len(ROUTE)}] -> {target_name} | x={x:.2f} y={y:.2f} z={z:.2f}")

        navigate_wait(x=x, y=y, z=z, frame_id="aruco_map", tolerance=tol)
        rospy.sleep(0.4 if is_center else 0.8)

        tag = f"{i:02d}_{target_name}_x{x:.2f}_y{y:.2f}"
        snap(tag)

    set_effect(effect="rainbow")
    rospy.loginfo("✅ Маршрут пройден, сажусь…")

finally:
    try:
        land()
    except Exception:
        pass