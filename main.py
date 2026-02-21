from aruco_frames_flight import ArucoFramesDroneController
from points_flight import PointsFlightDroneController
from lucky_flight import LuckyFlightDroneController
from recognize import RecognizeImage

ARUCO_MARKERS = [49, 81, 51, 50, 61, 50, 58, 62, 64]

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

def main():

    # 1. Полет по меткам
    frames = ArucoFramesDroneController(markers=ARUCO_MARKERS)
    frames.fly()

    # 2. Продвинутый полет по координатам
    body_new = PointsFlightDroneController(points=POINTS)
    body_new.fly()

    # 3. На удачу
    lucky = LuckyFlightDroneController(moves=MOVES)
    lucky.fly()

    # Распознавание
    recognize = RecognizeImage()
    recognize.start()
    

if __name__ == "__main__":
    main()