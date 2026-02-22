import math
import time
import rclpy

from drone_controller import DroneController

TAKEOFF_HEIGHT = 0.5
FLIGHT_SPEED = 0.5
TOLERANCE = 0.2
WAIT_TIMEOUT = 20.0

def finite(x: float) -> bool:
    return math.isfinite(x) and not math.isnan(x)

def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

class PointsFlightDroneController(DroneController):
    def __init__(self, points):
        super().__init__()
        self.points = points

    def navigate_wait(self, x, y, z):
        ok = self.navigate(x, y, z, speed=FLIGHT_SPEED, frame_id="body")
        if not ok:
            return False

        t0 = time.time()
        while time.time() - t0 < WAIT_TIMEOUT and rclpy.ok():
            telem = self.get_telemetry(frame_id="body")
            if not telem.get("connected"):
                self.sleep_spin(0.2)
                continue

            dx = telem["x"] - x
            dy = telem["y"] - y
            dz = telem["z"] - z
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            if dist < TOLERANCE:
                return True

            self.sleep_spin(0.2)

        self.get_logger().warn(f"Timeout waiting target: x={x} y={y} z={z}")
        return False


    def fly(self):
        self.get_logger().info("\n=== Points Script Started ===")
        
        self.sleep_spin(2)
        
        self.get_logger().info(f"\n>>> Taking off to {TAKEOFF_HEIGHT} meters")
        success = self.navigate(x=0.0, y=0.0, z=TAKEOFF_HEIGHT, speed=1.0, auto_arm=True, frame_id="body")
        if not success:
            self.get_logger().error("ERROR: Takeoff failed")
            return
        
        self.get_logger().info("Waiting for takeoff to complete...")
        self.sleep_spin(5)

        self.get_logger().info("Start flight by points")
        for i, (dx, dy, dz) in enumerate(self.points, start=1):
            self.get_logger().info(f"Moving to {i}: ({dx}, {dy}, {dz})")
            success = self.navigate_wait(dx, dy, dz) # или поменять на own_navigate_wait

            if success:
                self.get_logger().info(f"Success fly to {i}: ({dx}, {dy}, {dz})")
                self.shot()

                self.sleep_spin(5)
            else:
                self.get_logger().error(f"FAILED {i}: ({dx}, {dy}, {dz})")
            
        # 3. Посадка
        self.get_logger().info("\n>>> Landing")
        success = self.land()
        if success:
            self.get_logger().info("Landing command sent successfully")
            self.get_logger().info("Waiting for landing to complete...")
            self.sleep_spin(10)  # Ждем завершения посадки
        else:
            self.get_logger().error("ERROR: Landing command failed")
            
        self.get_logger().info("\n=== Flight completed successfully ===")
