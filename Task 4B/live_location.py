'''
*****************************************************************************************
*
*        		===============================================
*           		Geo Guide (GG) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 4B of Geo Guide (GG) Theme (eYRC 2023-24).
*
*  Team: GG-1949
*  Members: Akshit Gangwar, Ayush Karapagale, Prakhar Shukla, Sparsh Gautam
*****************************************************************************************
'''

# Importing necessary libraries
import cv2
import csv
import os
import numpy as np
import pandas as pd
from cv2 import aruco

# Initialising the aruco marker's dictionary. The markers on the arena are 4x4.
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
aruco_params = aruco.DetectorParameters()

# Initializing the video capture
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3, 1200)  # Setting width of the video capture
cap.set(4, 800)   # Setting height of the video capture

# Camera matrix and Distortion coefficients obtained after the camera calibration.
mtx = np.array([
    [1.26063217e+03, 0.00000000e+00, 9.87806100e+02],
    [0.00000000e+00, 1.35415601e+03, 5.05162014e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

dist = np.array([[-0.18516032, 0.89329069, -0.00102129, 0.0091498, -0.9055118]])

# Variables for cropping the arena.
x, y, width, height = 190, 0, 850, 1080
marker_info = {}  # Dictionary to store information about detected markers

# Loading latitude and longitude data from CSV
lat_lon_data = pd.read_csv('lat_long.csv')
fixed_marker_coordinates = {}

# Storing marker coordinates in a dictionary for reference
for index, row in lat_lon_data.iterrows():
    fixed_marker_coordinates[row['id']] = (row['lat'], row['lon'])

# Path to the CSV file where latitude, longitude, distance, and rotation angle will be written
csv_file_path = 'nearest2.csv'
fieldnames = ['Latitude', 'Longitude', 'Distance', 'RotationAngle']

# Main loop
while True:
    # Capturing a frame from the video feed
    ret, frame = cap.read()
    
    # Cropping the frame to focus on the arena
    frame = frame[y:y + height, x:x + width]
    
    # Correcting distortion using camera matrix and distortion coefficients
    frame = cv2.undistort(frame, mtx, dist)
    
    # Converting the frame to grayscale for marker detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detecting markers in the grayscale frame
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    marker_info.clear()  # Clearing previous marker information

    # Storing information about detected markers in the marker_info dictionary
    if ids is not None:
        for i in range(ids.size):
            center_x, center_y = np.mean(corners[i][0], axis=0)
            rect_points = corners[i][0].astype(int)
            marker_info[ids[i][0]] = {'center': (center_x, center_y), 'corners': corners[i][0]}

    # Adding placeholders for markers not detected in the current frame
    for marker_id in fixed_marker_coordinates.keys():
        if marker_id not in marker_info:
            marker_info[marker_id] = {'center': (np.nan, np.nan), 'corners': np.array([[np.nan, np.nan]])}

    # Processing data if ARUCO marker with ID 1 is detected
    if 1 in marker_info:
        marker_1_info = marker_info[1]
        nearest_aruco_id = min(
            (k for k in marker_info.keys() if k != 1),
            key=lambda k: np.linalg.norm(np.array(marker_info[k]['center']) - np.array(marker_1_info['center']))
        )

        if nearest_aruco_id in fixed_marker_coordinates:
            nearest_marker_coords = fixed_marker_coordinates[nearest_aruco_id]
            distance = np.linalg.norm(np.array(marker_1_info['center']) - np.array(nearest_marker_coords))
            x_diff = marker_1_info['corners'][0][0] - marker_1_info['corners'][1][0]
            y_diff = marker_1_info['corners'][0][1] - marker_1_info['corners'][1][1]
            rotation_angle = np.degrees(np.arctan2(y_diff, x_diff))
            rotation_angle = rotation_angle - 180
            # Writing data to CSV file
            with open(csv_file_path, 'w', newline='') as csvfile:
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                if os.stat(csv_file_path).st_size == 0:  # Check if the file is empty
                    writer.writeheader()
                writer.writerow({
                    'Latitude': nearest_marker_coords[0],
                    'Longitude': nearest_marker_coords[1],
                    'Distance': distance,
                    'RotationAngle': rotation_angle
                })
            
            # Printing information about the nearest marker
            print(f"Nearest ARUCO Marker ID: {nearest_aruco_id}")
            print(f"Nearest Latitude: {nearest_marker_coords[0]}")
            print(f"Nearest Longitude: {nearest_marker_coords[1]}")
            print(f"Distance to Nearest Marker: {distance}")
            print(f"Rotation Angle of Marker 1: {rotation_angle}")

    # Displaying the frame 
    cv2.namedWindow('GeoGuide', cv2.WINDOW_NORMAL)
    cv2.imshow('GeoGuide', frame)

    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
