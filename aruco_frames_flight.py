from drone_controller import DroneController

TAKEOFF_HEIGHT = 0.8
FLIGHT_SPEED = 0.5

FRAMES = [
    (1.6, 0.0, 0.5),
    (1.6, 1.6, 0.5),
    (0.8, 0.0, 0.5),
    (0.0, 0.0, 0.5),
    (0.0, 1.6, 0.5)
]

class ArucoFramesDroneController(DroneController):
    def __init__(self, markers):
        super().__init__()
        self.markers = markers

    def fly(self):
        self.get_logger().info("\n=== Aruco Frames Script Started ===")
        
        self.sleep_spin(2)
        
        self.get_logger().info(f"\n>>> Taking off to {TAKEOFF_HEIGHT} meters")
        success = self.navigate(x=0.0, y=0.0, z=TAKEOFF_HEIGHT, speed=1.0, auto_arm=True, frame_id="body")
        if not success:
            self.get_logger().error("ERROR: Takeoff failed")
            return
        
        self.get_logger().info("Waiting for takeoff to complete...")
        self.sleep_spin(5)

        self.get_logger().info("Start flight by targets")
        for i, (dx, dy, dz) in enumerate(FRAMES, start=1):

            success = self.navigate(x=dx, y=dy, z=0.5, speed=FLIGHT_SPEED, frame_id="aruco_map")
            if success:
                self.get_logger().info(f"Success fly to {i}")
                self.shot()
                self.sleep_spin(5)
            else: 
                self.get_logger().error(f"NOT FOUND {i}, skipping.")

            
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
