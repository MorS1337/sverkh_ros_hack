from drone_controller import DroneController

TAKEOFF_HEIGHT = 0.5
FLIGHT_SPEED = 0.5

class LuckyFlightDroneController(DroneController):
    def __init__(self, moves):
        super().__init__()
        self.moves = moves

    def fly(self):
        self.get_logger().info("\n=== Lucky Script Started ===")
        
        self.sleep_spin(2)
        
        self.get_logger().info(f"\n>>> Taking off to {TAKEOFF_HEIGHT} meters")
        success = self.navigate(x=0.0, y=0.0, z=TAKEOFF_HEIGHT, speed=1.0, auto_arm=True, frame_id="body")
        if not success:
            self.get_logger().error("ERROR: Takeoff failed")
            return
        
        self.get_logger().info("Waiting for takeoff to complete...")
        self.sleep_spin(5)

        self.get_logger().info("Start flight by moves")
        for i, (dx, dy, dz) in enumerate(self.moves, start=1):
            self.get_logger().info(f"Moving to {i}: ({dx}, {dy}, {dz})")
            success = self.navigate(dx, dy, dz, speed=FLIGHT_SPEED, frame_id="body")

            if success:
                self.get_logger().info(f"Success fly to {i}: ({dx}, {dy}, {dz})")
                self.shot()

                self.sleep_spin(10)
            else:
                self.get_logger().error(f"FAILED {i}: ({dx}, {dy}, {dz})")
            

        self.get_logger().info("\n>>> Landing")
        success = self.land()
        if success:
            self.get_logger().info("Landing command sent successfully")
            self.get_logger().info("Waiting for landing to complete...")
            self.sleep_spin(10)
        else:
            self.get_logger().error("ERROR: Landing command failed")
            
        self.get_logger().info("\n=== Flight completed successfully ===")
