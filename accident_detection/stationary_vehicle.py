import numpy as np
from collections import defaultdict
from accident_detection.logger import Logger

logger = Logger('stationary')

vehicle_history = defaultdict(list)
MOVEMENT_THRESHOLD = 20         

def check_stationary_status(tracker_id, center, fps, stationary_time):

    history = vehicle_history[tracker_id]
    history.append(center)
    STATIONARY_FRAMES = fps * stationary_time   
  
    if len(history) > STATIONARY_FRAMES:
        history.pop(0)
        
  
    if len(history) < STATIONARY_FRAMES:
        return False
        
    
    points = np.array(history)
    movement = np.linalg.norm(
        np.max(points, axis=0) - np.min(points, axis=0)
    )
    
    return movement < MOVEMENT_THRESHOLD