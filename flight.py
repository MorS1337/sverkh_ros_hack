import rospy
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from clover import srv
from std_srvs.srv import Trigger
import os

rospy.init_node('flight')
bridge = CvBridge()

get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
land = rospy.ServiceProxy('land', Trigger)

def navigate_wait(x, y, z, frame_id, tolerance=0.2):
    navigate(x=x, y=y, z=z, frame_id=frame_id, auto_arm=True)
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='navigate_target')
        if math.sqrt(telem.x ** 2 + telem.y ** 2 + telem.z ** 2) < tolerance:
            break
        rospy.sleep(0.2)

route = [
    (0.0, 0.0),  # 1. 49 (Старт)
    (0.8, 0.0),  # 2. 81
    (1.6, 0.0),  # 3. 51
    (1.6, 0.8),  # 4. 61
    (0.8, 0.8),  # 5. Центр (99)
    (0.0, 0.8),  # 6. 50
    (0.0, 1.6),  # 7. 58
    (0.8, 1.6),  # 8. 62
    (1.6, 1.6)   # 9. 64 (Финиш)
]

print("Взлетаем")
navigate_wait(x=0.0, y=0.0, z=0.8, frame_id='body')
rospy.sleep(2.0) 

for i, (x, y) in enumerate(route):
    print(f"Летим в клетку {i+1}/9: X={x}, Y={y}")
    navigate_wait(x=x, y=y, z=0.8, frame_id='aruco_map')
    
    rospy.sleep(2.0) 
    
    print("📸😂✌️")
    try:
        msg = rospy.wait_for_message('main_camera/image_raw', Image, timeout=5.0)
        frame = bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        filename = f"photos/cell_{i+1}_x{x}_y{y}.jpg"
        os.makedirs("photos", exist_ok=True)
        
        cv2.imwrite(filename, frame)
        print(f"✅ Сохранено: {filename}")
        
    except rospy.ROSException:
        print("❌ Ошибка: камера не ответила, летим дальше")

print("Лабиринт пройден. Посадка...")
land()