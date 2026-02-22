from aruco_frames_flight import ArucoFramesDroneController
from points_flight import PointsFlightDroneController
from lucky_flight import LuckyFlightDroneController
from recognize import RecognizeImage
import rclpy
import threading
import time

ARUCO_MARKERS = [49, 81, 51, 61, 50, 58, 62, 64]

CELL = 0.8
MOVES = [    
    (+CELL,     0.0,        0.0),
    (+CELL,     0.0,        0.0),
    (0.0,       +CELL,      0.0),
    (-CELL,     0.0,        0.0),
    (-CELL,     0.0,        0.0),
    (0.0,       +CELL,      0.0),
    (+CELL,     0.0,        0.0),
    (+CELL,     0.0,        0.0)
]

POINTS = [
    (0.8,   0.0,    0.0),
    (1.6,   0.0,    0.0),
    (1.6,   0.8,    0.0),
    (0.8,   0.8,    0.0),
    (0.0,   0.8,    0.0),
    (0.0,   1.6,    0.0),
    (0.8,   1.6,    0.0),
    (1.6,   1.6,    0.0)
]

def telemetry_loop(node):
    while rclpy.ok():
        try:
            node.get_telemetry()
            time.sleep(1.0) 
        except Exception as e:
            node.get_logger().error(f'Telemetry error: {str(e)}')
            time.sleep(2.0)


def main():
    rclpy.init()
    
    # 1. Полет по меткам
    strategy = ArucoFramesDroneController(markers=ARUCO_MARKERS)

    # 2. Продвинутый полет по координатам
    strategy = PointsFlightDroneController(points=POINTS)

    # 3. На удачу
    strategy = LuckyFlightDroneController(moves=MOVES)

    # Запускаем поток телеметрии
    telemetry_thread = threading.Thread(target=lambda: telemetry_loop(strategy), daemon=True)
    telemetry_thread.start()
    
    try:
        strategy.fly()
        
    except KeyboardInterrupt:
        print("\n>>> Operation interrupted by user")
        print("Attempting to land...")
        try:
            strategy.land()
            time.sleep(5)
        except:
            pass
        
    except Exception as e:
        print(f"ERROR: Unexpected error occurred: {str(e)}")
        print("Attempting emergency landing...")
        try:
            strategy.land()
        except:
            pass
            
    finally:
        print("\nShutting down...")
        strategy.destroy_node()
        rclpy.shutdown()

    # Распознавание
    recognize = RecognizeImage()
    recognize.start()
    

if __name__ == "__main__":
    main()