import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from cv2 import TrackerCSRT_create
import time

# === ROS Velocity Publisher Node ===
rospy.init_node('yolo_follower', anonymous=True)
pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)

# === Velocity Command Function ===
def send_velocity(x, z):
    msg = Twist()
    msg.linear.x = x
    msg.angular.z = z
    pub.publish(msg)

# === Load Class Names ===
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]

# === Load YOLOv4 Model ===
net = cv2.dnn.readNetFromDarknet("modelos/yolov4.cfg", "modelos/yolov4.weights")

# === Get Output Layers ===
layer_names = net.getLayerNames()
unconnected = net.getUnconnectedOutLayers()
if isinstance(unconnected[0], (list, np.ndarray)):
    output_layers = [layer_names[i[0] - 1] for i in unconnected]
else:
    output_layers = [layer_names[i - 1] for i in unconnected]

# === Distance Estimation Parameters ===
FOCAL_LENGTH = 800
KNOWN_WIDTHS = {"person": 0.5}

def estimate_distance(real_width, perceived_width):
    if perceived_width == 0:
        return None
    return (FOCAL_LENGTH * real_width) / perceived_width

# === Histogram Functions ===
def get_histogram(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    hist = cv2.calcHist([hsv], [0, 1], None, [50, 60], [0, 180, 0, 256])
    cv2.normalize(hist, hist)
    return hist

def compare_histograms(hist1, hist2):
    return cv2.compareHist(hist1, hist2, cv2.HISTCMP_CORREL)

# === Tracking Setup ===
DETECTION_INTERVAL = 30
tracker = None
initBB = None
frame_counter = 0
locked_hist = None
last_command_time = time.time()EOFError
reaction_time = 0.0

CENTER_THRESHOLD = 50
IDEAL_DISTANCE = 1.5
DISTANCE_THRESHOLD = 0.25

# === Start Video Stream ===
cap = cv2.VideoCapture(0)

command = ""
offset_text = ""
distance_text = ""

while not rospy.is_shutdown():
    start_time = time.time()
    ret, frame = cap.read()
    if not ret:
        break

    frame_counter += 1
    height, width, _ = frame.shape
    small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

    if tracker is None or frame_counter % DETECTION_INTERVAL == 0:
        tracker = None
        initBB = None

        blob = cv2.dnn.blobFromImage(small_frame, 1/255.0, (416, 416), swapRB=True, crop=False)
        net.setInput(blob)
        outs = net.forward(output_layers)

        boxes = []
        candidates = []

        for output in outs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]
                if confidence > 0.5 and classes[class_id] == "person":
                    center_x = int(detection[0] * (width // 2)) * 2
                    center_y = int(detection[1] * (height // 2)) * 2
                    w = int(detection[2] * (width // 2)) * 2
                    h = int(detection[3] * (height // 2)) * 2
                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)
                    boxes.append([x, y, w, h])
                    candidates.append((x, y, w, h))

        best_box = None
        best_score = -1
        for (x, y, w, h) in candidates:
            crop = frame[max(0,y):y+h, max(0,x):x+w]
            if crop.size == 0:
                continue
            hist = get_histogram(crop)
            if locked_hist is None:
                locked_hist = hist
                best_box = (x, y, w, h)
                break
            else:
                score = compare_histograms(locked_hist, hist)
                if score > best_score:
                    best_score = score
                    best_box = (x, y, w, h)

        if best_box is not None:
            initBB = best_box
            x, y, w, h = initBB
            distance = estimate_distance(KNOWN_WIDTHS["person"], w)
            distance_text = "{:.2f} m".format(distance) if distance else ""
            tracker = TrackerCSRT_create()
            tracker.init(frame, initBB)
    else:
        success, box = tracker.update(frame)
        if success:
            x, y, w, h = [int(v) for v in box]
            distance = estimate_distance(KNOWN_WIDTHS["person"], w)
            distance_text = "{:.2f} m".format(distance) if distance else ""
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)

            frame_center = width / 2
            box_center = x + w / 2
            horizontal_error = box_center - frame_center

            if horizontal_error < -CENTER_THRESHOLD:
                command = "Move Left"
            elif horizontal_error > CENTER_THRESHOLD:
                command = "Move Right"
            else:
                command = "Centered"

            if distance is not None:
                if distance > IDEAL_DISTANCE + DISTANCE_THRESHOLD:
                    command += ", Move Forward"
                elif distance < IDEAL_DISTANCE - DISTANCE_THRESHOLD:
                    command += ", Move Backward"
                else:
                    command += ", Stop"
            else:
                command += ", No Distance"

            angular_z = 0.4 if "Move Left" in command else -0.4 if "Move Right" in command else 0.0
            linear_x = 0.3 if "Move Forward" in command else -0.2 if "Move Backward" in command else 0.0
            send_velocity(linear_x, angular_z)

            reaction_time = time.time() - last_command_time
            last_command_time = time.time()

            offset_text = "Offset: {:.0f}px".format(horizontal_error)
        else:
            tracker = None
            command = "Lost Target"
            offset_text = ""
            distance_text = ""

    fps = 1.0 / (time.time() - start_time)

    # Display all relevant info at top-left
    cv2.putText(frame, f"FPS: {fps:.2f}", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    cv2.putText(frame, f"Reaction Time: {reaction_time:.2f}s", (10, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
    cv2.putText(frame, f"Distance: {distance_text}", (10, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, offset_text, (10, 95),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
    cv2.putText(frame, f"Command: {command}", (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    cv2.imshow("Detector and Tracker with Identity Lock", frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()

