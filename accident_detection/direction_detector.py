from collections import defaultdict, deque
import math


class VehicleDirectionTracker:

    def __init__(self, history_len=20, pixel_threshold=3):
        
        self.direction_history = defaultdict(
            lambda: deque(maxlen=history_len)
        )
        self.vehicle_angles = {}
        self.pixel_threshold = pixel_threshold

    def _calculate_direction_change(self, points):
        
        if len(points) < 15:
            return 0.0

        # Vector A (Past Direction)
        dx_before = points[-8][0] - points[-15][0]
        dy_before = points[-8][1] - points[-15][1]

        # Vector B (Current Direction)
        dx_after = points[-1][0] - points[-7][0]
        dy_after = points[-1][1] - points[-7][1]

        
        if (abs(dx_before) + abs(dy_before) < self.pixel_threshold) or (
            abs(dx_after) + abs(dy_after) < self.pixel_threshold
        ):
            return 0.0

        angle_before = math.degrees(math.atan2(dy_before, dx_before))
        angle_after = math.degrees(math.atan2(dy_after, dx_after))

        angle_difference = abs(angle_after - angle_before)
        if angle_difference > 180:
            angle_difference = 360 - angle_difference

        return angle_difference

    def start_frame_cleanup(self, active_ids):

        tracked_ids = list(self.direction_history.keys())
        for tracker_id in tracked_ids:
            if tracker_id not in active_ids:
                del self.direction_history[tracker_id]
                if tracker_id in self.vehicle_angles:
                    del self.vehicle_angles[tracker_id]

    def update_vehicle(self, tracker_id, box_center):
        self.direction_history[tracker_id].append(box_center)
        
        angle = self._calculate_direction_change(
            self.direction_history[tracker_id]
        )

        if angle is not None:
            self.vehicle_angles[tracker_id] = angle
            return angle 
            
        return 0.0