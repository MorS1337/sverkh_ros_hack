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

PHOTOS_DIR = "photos"
os.makedirs(PHOTOS_DIR, exist_ok=True)

TAKEOFF_Z = 1.5
WORK_Z = 0.8
CENTER_Z = 1.4   

CENTER_X = 0.8
CENTER_Y = 0.8

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

# Матрица:
# [[58,62,64],
#  [50, None,61],
#  [49, 81, 51]]
# Змейка с 49:
ROUTE = [
    ("aruco", 49),
    ("aruco", 81),
    ("aruco", 51),
    ("aruco", 61),
    ("center", None),  
    ("aruco", 50),
    ("aruco", 58),
    ("aruco", 62),
    ("aruco", 64),
]

try:
    set_effect(r=0, g=0, b=255)
    rospy.sleep(0.5)

    navigate_wait(z=TAKEOFF_Z, frame_id="body", auto_arm=True, tolerance=0.15)
    rospy.sleep(2.0)

    if not wait_for_aruco_map(timeout_s=10.0):
        set_effect(effect="blink_fast", r=255, g=0, b=0)
        raise RuntimeError("aruco_map не появился (дрон не видит карту)")

    navigate_wait(z=WORK_Z, frame_id="body", tolerance=0.15)
    rospy.sleep(1.0)

    for idx, step in enumerate(ROUTE, start=1):
        kind, val = step

        if kind == "aruco":
            mid = val
            rospy.loginfo(f"[{idx}/{len(ROUTE)}] -> aruco_{mid}")
            navigate_wait(x=0, y=0, z=WORK_Z, frame_id=f"aruco_{mid}", tolerance=0.15)
            rospy.sleep(0.8)
            snap(f"{idx:02d}_aruco_{mid}")

        else:
            rospy.loginfo(f"[{idx}/{len(ROUTE)}] -> CENTER (aruco_map x={CENTER_X}, y={CENTER_Y})")

            if not wait_for_aruco_map(timeout_s=2.0):
                rospy.logwarn("aruco_map пропал, пробую восстановиться через aruco_61…")
                navigate_wait(x=0, y=0, z=1.2, frame_id="aruco_61", tolerance=0.2)
                rospy.sleep(1.0)

            navigate_wait(x=CENTER_X, y=CENTER_Y, z=CENTER_Z, frame_id="aruco_map", tolerance=0.25)
            rospy.sleep(0.3) 
            snap(f"{idx:02d}_CENTER_x{CENTER_X}_y{CENTER_Y}")

    set_effect(effect="rainbow")
    rospy.loginfo("✅ Маршрут пройден")

finally:
    try:
        land()
    except Exception:
        pass