from accident_detection.logger import Logger

logger = Logger('collision_detect')

def calculate_iou(box1, box2):
    x_left = max(box1[0], box2[0])
    y_top = max(box1[1], box2[1])
    x_right = min(box1[2], box2[2])
    y_bottom = min(box1[3], box2[3])

    if x_right <= x_left or y_bottom <= y_top:
        return 0.0

    intersection = (x_right - x_left) * (y_bottom - y_top)
    area1 = (box1[2] - box1[0]) * (box1[3] - box1[1])
    area2 = (box2[2] - box2[0]) * (box2[3] - box2[1])
    
    union = area1 + area2 - intersection
    return intersection / union if union > 0 else 0.0


def batch_collision_check(detections, frame, classes, iou_threshold):
    colliding_ids = set()
    
    if detections.tracker_id is None or len(detections.xyxy) < 2:
        return colliding_ids

    boxes = detections.xyxy
    tracker_ids = detections.tracker_id
    class_ids = detections.class_id
    n = len(boxes)

    centers_x = []
    centers_y = []
    widths = []
    heights = []
    valid_mask = []

    HORIZON_Y_LIMIT = 120 

    for i in range(n):
        b = boxes[i]
        w = b[2] - b[0]
        h = b[3] - b[1]
        cls = int(class_ids[i])
        tid = tracker_ids[i]
        
        is_too_small = (w < 25) or (h < 25)
        is_in_horizon = (b[3] < HORIZON_Y_LIMIT)
        
        if is_too_small or is_in_horizon or cls not in classes or tid is None or tid < 0:
            valid_mask.append(False)
        else:
            valid_mask.append(True)

        centers_x.append((b[0] + b[2]) / 2)
        centers_y.append((b[1] + b[3]) / 2)
        widths.append(w)
        heights.append(h)

    for i in range(n):
        if not valid_mask[i]:
            continue
            
        id1 = tracker_ids[i]
        cx1, cy1, w1, h1 = centers_x[i], centers_y[i], widths[i], heights[i]
        box1 = boxes[i]
        
        for j in range(i + 1, n):
            if not valid_mask[j]:
                continue
                
            id2 = tracker_ids[j]
            if id1 == id2:
                continue

            
            if abs(cx1 - centers_x[j]) > (w1 + widths[j]) / 2:
                continue
            if abs(cy1 - centers_y[j]) > (h1 + heights[j]) / 2:
                continue

           
            iou = calculate_iou(box1, boxes[j])
            
            if iou >= 0.80:
                continue

            if iou >= iou_threshold:
                colliding_ids.add(id1)
                colliding_ids.add(id2)
                
    return colliding_ids
