# Просто пакеты
import threading, sys, time, os

# Управление дроном
import rclpy
from rclpy.node import Node
from offboard_interfaces.srv import Navigate, GetTelemetry
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Для камеры
import cv2


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Сервисы
        self.navigate_client = self.create_client(Navigate, '/navigate')
        self.land_client = self.create_client(Trigger, '/land')
        self.telemetry_client = self.create_client(GetTelemetry, '/get_telemetry')

        # Для камеры
        self.bridge = CvBridge()
        self._last_frame = None
        self.found_objects = set()
        self.create_subscription(Image, "/aruco_det/debug_image", self._image_callback, 10)
        
        # Ждем готовности сервисов
        self._wait_for_services()
        
        self.get_logger().info('All services are ready')
        self._lock = threading.Lock()
        

    def _image_callback(self, msg):
        try:
            self._last_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Frame saving error: {e}")


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


    def _wait_for_services(self):
        services = [
            (self.navigate_client, 'Navigate'),
            (self.land_client, 'Land'),
            (self.telemetry_client, 'Telemetry')
        ]
        
        for client, name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                if not rclpy.ok():
                    sys.exit("ROS2 interrupted, exiting...")
                self.get_logger().info(f'{str(client)} service not available, waiting...')


    def _call_service(self, client, request):
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


    def get_telemetry(self):
        request = GetTelemetry.Request()
        response = self._call_service(self.telemetry_client, request)
        
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
        
    def get_telemetry(self, frame_id):
        req = GetTelemetry.Request()
        if frame_id is not None and hasattr(req, "frame_id"):
            req.frame_id = str(frame_id)
        resp = self._call(self.tel_cli, req, timeout=3.0)
        if resp and getattr(resp, "connected", False):
            return resp
        return None


    def navigate(self, x, y, z, yaw=float("nan"), speed=0.5, auto_arm=False, angular_velocity=0.0, frame_id="body"):
        request = Navigate.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        request.yaw = float(yaw)
        request.speed = float(speed)
        request.auto_arm = bool(auto_arm)
        request.angular_velocity = float(angular_velocity)
        request.frame_id = str(frame_id)
        
        response = self._call_service(self.navigate_client, request)
        if response:
            self.get_logger().info(f'Navigate: success={response.success}, message="{response.message}"')
            return response.success
        else:
            self.get_logger().error("Navigate timeout")
            return False

    def land(self):
        request = Trigger.Request()
        response = self._call_service(self.land_client, request)
        if response:
            self.get_logger().info(f'Land: success={response.success}, message="{response.message}"')
            return response.success
        else:
            self.get_logger().error("Land timeout")
            return False

    def sleep_spin(self, sec: float, step=0.1):
        t0 = time.time()
        while time.time() - t0 < sec and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=step)
            