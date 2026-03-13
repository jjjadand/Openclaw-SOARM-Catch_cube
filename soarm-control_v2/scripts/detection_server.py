import cv2
import time
import threading
import argparse
import json
from flask import Flask, jsonify, Response
from ultralytics import YOLO

# Global variables to store detection results
detection_lock = threading.Lock()
current_detection = {
    "timestamp": 0,
    "objects": []
}

# Flask application
app = Flask(__name__)

def parse_args():
    parser = argparse.ArgumentParser(description="YOLOv11 Detection Server")
    parser.add_argument("--weights", type=str, default="/home/seeed/detection/runs/detect/runs/train/exp3/weights/best.pt", help="Path to model weights")
    parser.add_argument("--source", type=int, default=0, help="Camera index")
    parser.add_argument("--conf", type=float, default=0.25, help="Confidence threshold")
    parser.add_argument("--port", type=int, default=5000, help="Server port")
    parser.add_argument("--host", type=str, default="0.0.0.0", help="Server host")
    return parser.parse_args()

def detection_loop(args):
    global current_detection
    
    print(f"[INFO] Loading model: {args.weights}")
    try:
        model = YOLO(args.weights)
    except Exception as e:
        print(f"[ERROR] Failed to load model: {e}")
        return

    print(f"[INFO] Opening camera {args.source}...")
    cap = cv2.VideoCapture(args.source)
    # Set resolution for better performance/quality balance
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera {args.source}")
        return

    print("[INFO] Detection loop started")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("[ERROR] Failed to read frame")
            time.sleep(1)
            continue

        # Run inference
        results = model.predict(frame, conf=args.conf, verbose=False, stream=True)

        # Process results
        objects_list = []
        
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # Get coordinates
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                
                # Calculate center
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                
                # Get class and confidence
                cls_id = int(box.cls[0])
                label = model.names[cls_id]
                conf = float(box.conf[0])

                objects_list.append({
                    "label": label,
                    "confidence": float(f"{conf:.2f}"),
                    "center": {"x": cx, "y": cy},
                    "bbox": {"x1": int(x1), "y1": int(y1), "x2": int(x2), "y2": int(y2)}
                })

        # Update global state
        with detection_lock:
            current_detection = {
                "timestamp": time.time(),
                "count": len(objects_list),
                "objects": objects_list
            }
        
        # Small sleep to prevent 100% CPU usage if inference is very fast
        # time.sleep(0.01)

@app.route('/coordinates', methods=['GET'])
def get_coordinates():
    with detection_lock:
        return jsonify(current_detection)

@app.route('/', methods=['GET'])
def index():
    return "YOLOv11 Detection Server is Running. Use <a href='/coordinates'>/coordinates</a> to get data."

if __name__ == "__main__":
    args = parse_args()
    
    # Start detection thread
    t = threading.Thread(target=detection_loop, args=(args,))
    t.daemon = True
    t.start()
    
    # Start Flask server
    print(f"[INFO] Starting server on http://{args.host}:{args.port}")
    app.run(host=args.host, port=args.port, debug=False, use_reloader=False)
