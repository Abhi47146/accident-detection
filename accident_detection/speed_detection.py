from collections import defaultdict, deque
import math
import time
from accident_detection.logger import Logger

logger = Logger('speed_detection')

# Stores the last 10 center coordinates of every vehicle
vehicle_history = defaultdict(lambda: deque(maxlen=10))


def update_vehicle_history(tracker_id, box):

    cx, cy = box
    timestamp=time.time()

    vehicle_history[tracker_id].append((cx, cy,timestamp))

def calculate_speed(points):

    if len(points) < 10:
        return None

    before_speeds = []
    after_speeds = []

    for i in range(4):

        x1, y1, t1 = points[i]
        x2, y2, t2 = points[i + 1]

        distance = math.sqrt(
            (x2-x1)**2 +
            (y2-y1)**2
        )

        dt = t2 - t1

        if dt > 0:
            before_speeds.append(distance/dt)

    for i in range(5,9):

        x1, y1, t1 = points[i]
        x2, y2, t2 = points[i + 1]

        distance = math.sqrt(
            (x2-x1)**2 +
            (y2-y1)**2
        )

        dt = t2 - t1

        if dt > 0:
            after_speeds.append(distance/dt)

    if len(before_speeds)==0 or len(after_speeds)==0:
        return None

    avg_before = sum(before_speeds)/len(before_speeds)

    avg_after = sum(after_speeds)/len(after_speeds)

    drop = avg_before - avg_after
    if avg_before ==0:
        drop_percent = 0.0
    else:
        drop_percent = (drop/avg_before)*100
    if drop_percent < 0:
        drop_percent = 0.0
    return avg_before, avg_after, drop_percent


def detect_speed(tracker_id, center):

    update_vehicle_history(tracker_id, center)
    result = calculate_speed(
    vehicle_history[tracker_id]
    )

    if result is not None:
        return result


    return 0,0,0