"""
* Team Id : GG_1949
* Author List : Ayush Karapagale, Akshit Gangawar, Prakhar Shukla, Sparsh Gautam
* Filename: nearimg.py
* Theme: Geo Guide -- EYRC
* Functions: calculate_distance, complete_path
* Global Variables: arena, model, classNames, priority_order, fixed_boxes, label_counts, aruco_dict, aruco_params, aruco_marker_coordinates, mtx, dist
"""

# Import necessary libraries
import cv2
import math
import numpy as np
from ultralytics import YOLO
from cv2 import aruco
from path_planning import complete_path

#Variable Description for non obvious Variables
"""
    1.mtx and dist: Used in camera calibration to correct lens distortion.
    2.label_frames: Aggregates detected labels per frame to discern the most prevalent event in each area.
    3.fixed_boxes: Relates specific areas (A, B, C, etc.) to their closest ArUco markers, essential for correlating detected events to physical locations in the arena.
    4.priority_order: Influences the routing by dictating the sequence in which events are addressed, based on provided priority.
"""

# Initialize YOLO model for event detection
# Google Drive Link for the model: 
model = YOLO("best888.pt")

# Define class names and priority order for events
classNames = ['Combat', 'Destroyed buildings', 'Fire', 'Military Vehicles', 'Humanitarian Aid and rehabilitation']
priority_order = ['Fire', 'Destroyed buildings', 'Humanitarian Aid and rehabilitation', 'Military Vehicles', 'Combat']

# Define dictionaries and variables for fixed boxes, label counts, and aruco markers
fixed_boxes = {'A': 21, 'B': 29, 'C': 30, 'D': 34, 'E': 48}         #ArUco markers nearest to respective event box
label_counts = {key: 0 for key in fixed_boxes.keys()}  
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
aruco_params = aruco.DetectorParameters()

"""
Function Name: calculate_distance
Input: point1, point2 - These are tuples or lists containing the x and y coordinates of two points respectively.
Output: Returns the Euclidean distance between the two points.
Logic: This function calculates the square root of the sum of the squared differences between the x and y coordinates of two points.
Example Call: calculate_distance((1, 2), (4, 6))
"""

def calculate_distance(point1, point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)

# Dictionary to store aruco marker coordinates
aruco_marker_coordinates = {}

# Open video capture
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3, 1920)
cap.set(4, 1080)
x, y, width, height = 200, 0, 1350, 1080

# Camera matrix and distortion coefficients
mtx = np.array([
    [1.26063217e+03, 0.00000000e+00, 9.87806100e+02],
    [0.00000000e+00, 1.35415601e+03, 5.05162014e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

dist = np.array([[-0.18516032, 0.89329069, -0.00102129, 0.0091498, -0.9055118]])

# Start continuous aruco marker detection
while len(aruco_marker_coordinates) < len(fixed_boxes):
    # Capturing a frame from the video feed
    success, img = cap.read()
    img = img[y:y+height, x:x+width]
    # Undistort the image
    img = cv2.undistort(img, mtx, dist)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    aruco.drawDetectedMarkers(img, corners, ids)

    if ids is not None:
        for i in range(len(ids)):
            marker_id = ids[i][0]
            if marker_id in fixed_boxes.values() and marker_id not in aruco_marker_coordinates:
                aruco_marker_coordinates[marker_id] = corners[i][0][0]

# Dictionary to store labels detected in each frame
label_frames = {}

# Run object detection for 50 frames
for frame_count in range(50):
    success, img = cap.read()
    img = img[y:y+height, x:x+width]
    
    # Detect objects using YOLO
    results = model(img, agnostic_nms=True, verbose=False)

    for r in results:
        boxes = r.boxes

        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            confidence = math.ceil((box.conf[0] * 100)) / 100

            label = ''
            labelll = ''

            if confidence > 0.6:
                cls = int(box.cls[0])
                label = classNames[cls]

                object_center = ((x1 + x2) // 2, (y1 + y2) // 2)

                nearest_marker = min(aruco_marker_coordinates.keys(),
                                     key=lambda marker_id: calculate_distance(object_center, aruco_marker_coordinates[marker_id]))

                image_name = [key for key, value in fixed_boxes.items() if value == nearest_marker][0]
                

                label_counts[image_name] += 1
                if frame_count not in label_frames:
                    label_frames[frame_count] = {}

                if image_name not in label_frames[frame_count]:
                    label_frames[frame_count][image_name] = []

                label_frames[frame_count][image_name].append(label)

                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                font = cv2.FONT_HERSHEY_SIMPLEX
                color = (0, 255, 0)
                thickness = 2

                #Changing names for labelling the bounding boxes
                if label == 'Fire':
                    labelll = 'fire'
                if label == 'Destroyed buildings':
                    labelll = 'destroyed_buildings'
                if label == 'Combat':
                    labelll = 'combat'
                if label == 'Humanitarian Aid and rehabilitation':
                    labelll = 'humanitarian_aid'
                if label == 'Military Vehicles':
                    labelll = 'military_vehicles'
                

                cv2.putText(img, labelll, (x1, y1 - 10), font, 0.6, color, thickness)

    cv2.namedWindow('GeoGuide', cv2.WINDOW_NORMAL)
    cv2.imshow('GeoGuide', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Dictionary to store final labels for each marker
final_labels_dict = {}

# Assign labels based on maximum occurrences for each image
for frame, image_labels in label_frames.items():
    for image_name, labels in image_labels.items():
        final_label = max(set(labels), key=labels.count)
        if image_name not in final_labels_dict:
            final_labels_dict[image_name] = final_label
        else:
            current_label_index = priority_order.index(final_labels_dict[image_name])
            new_label_index = priority_order.index(final_label)
            if new_label_index < current_label_index:
                final_labels_dict[image_name] = final_label

# Filter out labels with less than 20 occurrences
final_labels_dict_filtered = {}
for image_name, label in final_labels_dict.items():
    if label_counts[image_name] >= 20:
        final_labels_dict_filtered[image_name] = label

print(final_labels_dict_filtered)
# Sort keys based on priority order
sorted_keys = sorted(final_labels_dict_filtered.keys(), key=lambda k: priority_order.index(final_labels_dict[k]))

"""
Function Name: complete_path
Input: 
- sorted_keys: A list of keys sorted based on the given priority, representing specific nodes in the graph.
Output: Returns a list representing the path through the arena based on the priority of events.
Logic: This function calculates the optimal route through a graph based on given priorities.
Example Call: priority_route(arena, ['A', 'B', 'C'])
"""

# Find path based on priority route
path = complete_path(sorted_keys)

# Write path to a file
with open("path.txt", "w") as file:
    file.write(str(path))

# Write events to a file
with open("events.txt", "w") as file:
    file.write(str(sorted_keys))

# Release video capture and close windows
cap.release()
cv2.destroyAllWindows()
