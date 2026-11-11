import cv2
import os
from ultralytics import YOLO
import time
from datetime import datetime
import traceback
import supervision as sv
from multiprocessing import Process, Queue
from accident_detection.collision import batch_collision_check
from configparser import ConfigParser
import ast
from accident_detection.speed_detection import detect_speed
from accident_detection.direction_detector import VehicleDirectionTracker
from accident_detection.stationary_vehicle import check_stationary_status
from accident_detection.carriageway import get_carriageway
from accident_detection.accident_score import calculate_accident_score
from accident_detection.data_sender import send_to_api
from accident_detection.logger import Logger

logger = Logger('main')

def reconnect_to_camera(source_path):
    while True:
        video_capture = cv2.VideoCapture(source_path)
        if video_capture.isOpened():
            logger.debug(f"Successfully reconnected to camera {source_path}")
            return video_capture
        else:
            logger.error(f"Failed to reconnect to camera. Retrying in 5 seconds... {source_path}")
            time.sleep(5)


def process_video(stream_id, source_path, target_path, arm_id, flag, classes, custom_names, frame_queue, result_queue, fps, api, iou_threshold, line_config, stationary_time):

    try:
        video_capture = reconnect_to_camera(source_path)
        
        fps = int(fps)
        print("value of frame_speed", fps)
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        video_writer = cv2.VideoWriter(target_path, fourcc, fps, (640, 640))
        tracker = sv.ByteTrack()
        dir_tracker = VehicleDirectionTracker()
        iou_threshold = float(iou_threshold)
        x1, y1, x2, y2 = map(int, line_config)
        
    except Exception as err:
        logger.error(f"Error: {err} | Traceback: {traceback.format_exc()}")
        return

    try:
        while True:
            ret, frame = video_capture.read()
            if not ret:
                logger.error(f"Lost connection to camera {source_path}. Attempting to reconnect...")
                video_capture.release()
                video_capture = reconnect_to_camera(source_path)
                continue
            
            frame = cv2.resize(frame, (640, 640))
            frame_queue.put((stream_id, frame))  
            
            stream_id_recv, detections = result_queue.get()  

            if stream_id_recv != stream_id:
                logger.error(f"Stream ID mismatch: expected {stream_id}, got {stream_id_recv}")
                continue
            
            cv2.line(frame, (x1, y1), (x2, y2), color=(255, 255, 0), thickness=2)
            detections = tracker.update_with_detections(detections)
            collision_detected = False
            speed_detected = False
            direction_detected = False
            smoke_detected = False
            current_frame_stationary_count = 0
            left_vehicle_drop = []
            right_vehicle_drop = []
            
            active_ids = set(detections.tracker_id) if detections.tracker_id is not None else set()
            dir_tracker.start_frame_cleanup(active_ids)
           
            colliding_vehicles = batch_collision_check(detections, frame, classes, iou_threshold)
            collision_detected = len(colliding_vehicles) >= 2
            
            for class_id, tracker_id, xyxy in zip(detections.class_id, detections.tracker_id, detections.xyxy):           
                class_id = int(class_id)
                if class_id not in classes:
                    continue
                    
                center_x = int((xyxy[0] + xyxy[2]) / 2)
                center_y = int((xyxy[1] + xyxy[3]) / 2)
                carriageway = get_carriageway((center_x, center_y),line_config)
            
                box_color = (0, 255, 0)
                
                if collision_detected and tracker_id in colliding_vehicles:
                    print(f"Verified Match: ID #{tracker_id} is in a confirmed collision state.")
                    box_color = (0, 0, 255)
                    cv2.putText(frame, "COLLISION", (int(xyxy[0]), int(xyxy[1]) - 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)
                
                speed_before, speed_after, drop = detect_speed(tracker_id, (center_x, center_y))
                
                if speed_before != 0:
                    if carriageway == "LEFT":
                        left_vehicle_drop.append(drop)
                    else:
                        right_vehicle_drop.append(drop)
                
                if check_stationary_status(tracker_id, (center_x, center_y), fps, stationary_time):
                    current_frame_stationary_count += 1
                    cv2.putText(frame, "STATIONARY", (int(xyxy[0]), int(xyxy[1]) - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                current_angle_change = dir_tracker.update_vehicle(tracker_id, (center_x, center_y))
                if current_angle_change is not None and current_angle_change > 30:
                    direction_detected = True
                
                if current_angle_change is not None:
                    cv2.putText(frame, f"Dir:{current_angle_change:.1f}", (int(xyxy[0]), int(xyxy[1]) - 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
                    
                cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), box_color, 2)
                cv2.putText(frame, f"#{tracker_id} {custom_names[class_id]}", (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                if len(left_vehicle_drop) > 0:
                    left_avg_drop = sum(left_vehicle_drop) / len(left_vehicle_drop)
                else:
                    left_avg_drop = 0
                if len(right_vehicle_drop) > 0:
                    right_avg_drop = sum(right_vehicle_drop) / len(right_vehicle_drop)
                else:
                    right_avg_drop = 0

                if left_avg_drop > 50 or right_avg_drop > 50:
                    speed_detected = True

                score = calculate_accident_score(collision_detected,speed_detected,direction_detected,smoke_detected)
                if score >= 0.8:
                    vbv_file = {
                        "event_type": "STL1Z1",
                        "vids_scn": arm_id,
                        "timestamp": str(datetime.now().strftime("%Y-%m-%d %H:%M:%S")),
                        "class_name": custom_names[class_id],
                        "tracker_id": str(tracker_id),
                        "speed": None,
                        "camera_id": arm_id,
                        "image": None,
                        }
                    send_to_api(vbv_file, api)
            
            #cv2.putText(frame, f"No of stationary Vehicles: {current_frame_stationary_count}", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)  
            
            # if score >= 0.8:
            #     cv2.putText(frame, f"Accident score: {score:.2f}%", (150,150),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0),2)
            # cv2.putText(frame,f"Left Avg Drop : {left_avg_drop:.2f}%",(20, 20),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0),2)
            # cv2.putText(frame,f"Right Avg Drop : {right_avg_drop:.2f}%",(20, 50),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,0,0),2)
            if flag:
                video_writer.write(frame)

            cv2.imshow("Vehicle Counting (MOG2 + ROI)", frame)
            if cv2.waitKey(30) & 0xFF == 27:
                break

    except Exception as err:
        logger.error(f"Error: {err} | Traceback: {traceback.format_exc()}")
    finally:
        video_capture.release()
        video_writer.release()

def inference_worker(frame_queues,result_queues,device,model_id,yolo,openvino,engine,ncnn):
    
    model_filenames = {
        1: yolo,
        2: engine,
        3: openvino,
        4: ncnn
    }
    model_path = os.path.join(os.path.dirname(__file__), model_filenames[model_id])

    print("model_path:",model_path)

    if model_id == 1:   # PyTorch model (.pt)
        if device == 'cuda':
            trt_model = YOLO(model_path).to(device)
        else:
            trt_model = YOLO(model_path)

    else:
        # TensorRT / OpenVINO / NCNN models
        trt_model = YOLO(model_path, task="detect")

    while True:
        for stream_id, frame_queue in frame_queues.items():
            if not frame_queue.empty():
                stream_id_recv, frame = frame_queue.get()  
                results = trt_model(frame)[0]
                detections = sv.Detections.from_ultralytics(results)
                result_queues[stream_id].put((stream_id_recv, detections))  # Send stream_id and detections back



def main():
    print(f"os.outputcount {os.cpu_count()}")
    conf = ConfigParser()
    conf.read('/home/abhishek/Downloads/accident_detection/config.ini')
    junction_scns = conf.get("Junctions", "scns").split(',')
    debug_flag = conf.getboolean("video_debug", "flag")
    device= conf.get("models", "device")
    print(f"device is {device}")
    model_id=conf.getint("models","model_id")
    classes = conf.get("classes", "id").split(',')
    fps = conf.get("frame_speed", "fps")
    iou_threshold = conf.get("threshold", "iou_threshold")
    stationary_time = conf.getint("threshold","stationary_time_threshold")
    class_ids = [int(class_id) for class_id in classes]
    results_videos = conf.get("video_results", "pathnames").split(',')
    line_config = conf.get("zone_config", "line").split(',')
    custom_names = ast.literal_eval(conf.get("class_name", "custom_names"))
    api = conf.get("api", "api_url")
    yolo = conf.get("yolo_pt", "pt")
    openvino = conf.get("yolo_openvino", "openvino")
    engine = conf.get("yolo_engine", "engine")
    ncnn = conf.get("yolo_ncnn", "ncnn")
    
    frame_queues = {}
    result_queues = {}
    processes = []

    for i in range(len(junction_scns)):
        frame_queues[i] = Queue()
        result_queues[i] = Queue()

    # Start the inference worker process
    inference_process = Process(target=inference_worker, args=(frame_queues,result_queues,device,model_id,yolo,openvino,engine,ncnn))
    inference_process.start()
    processes.append(inference_process)

    # Start the video processing processes
    for i in range(len(junction_scns)):
        video_stream = conf.get(junction_scns[i], 'stream')
        p = Process(target=process_video, args=(i, video_stream, results_videos[i], junction_scns[i], debug_flag, class_ids, custom_names, frame_queues[i], result_queues[i], fps, api, iou_threshold, line_config, stationary_time))
        p.start()
        processes.append(p)

    for p in processes:
        p.join()

if __name__ == "__main__":
    main()