# Import necessary libraries
import socket
import struct
import cv2
import numpy as np
import pandas as pd
from cv2 import aruco
import time
import csv
import os

# Function to get the IP address of the server
def get_ip_address():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(('10.255.255.255', 1))
        IP = s.getsockname()[0]
    except Exception:
        IP = '127.0.0.1'
    finally:
        s.close()
    return IP

# Initialize socket for communication
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('', 8080)  # Binding to all available interfaces
sock.bind(server_address)
sock.listen(1)

# Print server's IP address and wait for a connection
print(f"Server IP Address: {get_ip_address()}")
print("Waiting for a connection...")
connection, client_address = sock.accept()
print(f"Connection from {client_address}")

# Send path and event data to the client
with open("path.txt", "r") as file:
    path_data = file.read().strip()
    connection.sendall((path_data + "\n").encode('utf-8'))

with open("events.txt", "r") as file:
    events_data = file.read().strip()
    connection.sendall((events_data + "\n").encode('utf-8'))

# Initialize ARUCO marker detection
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_1000)
aruco_params = aruco.DetectorParameters()

# Initialize video capture
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3, 1400)  # Setting width of the video capture
cap.set(4, 900)   # Setting height of the video capture

# Camera matrix and distortion coefficients for distortion correction
mtx = np.array([
    [1.26063217e+03, 0.00000000e+00, 9.87806100e+02],
    [0.00000000e+00, 1.35415601e+03, 5.05162014e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
])

dist = np.array([[-0.18516032, 0.89329069, -0.00102129, 0.0091498, -0.9055118]])

# Dictionary to store marker information
marker_info = {}

# Load latitude and longitude data from CSV
lat_lon_data = pd.read_csv('lat_long.csv')
fixed_marker_coordinates = {}

# Store marker coordinates in a dictionary for reference
for index, row in lat_lon_data.iterrows():
    fixed_marker_coordinates[row['id']] = (row['lat'], row['lon'])

# Path to the CSV file where data will be written
csv_file_path = 'nearest2.csv'
fieldnames = ['Latitude', 'Longitude', 'Distance', 'RotationAngle']

# Main loop for video processing
while True:
    ret, frame = cap.read()
    frame = cv2.undistort(frame, mtx, dist)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)
    marker_info.clear()  

    # Store information about detected markers in the marker_info dictionary
    if ids is not None:
        for i in range(ids.size):
            center_x, center_y = np.mean(corners[i][0], axis=0)
            rect_points = corners[i][0].astype(int)
            marker_info[ids[i][0]] = {'center': (center_x, center_y), 'corners': corners[i][0]}

    # Add placeholders for markers not detected in the current frame
    for marker_id in fixed_marker_coordinates.keys():
        if marker_id not in marker_info:
            marker_info[marker_id] = {'center': (np.nan, np.nan), 'corners': np.array([[np.nan, np.nan]])}

    # Process data if ARUCO marker with ID 100 is detected
    if 100 in marker_info:
        marker_100_info = marker_info[100]
        pixel_coordinates = f"{marker_100_info['center'][0]},{marker_100_info['center'][1]}"

        print(f"Pixel Coordinates of Marker 100 Center: ({marker_100_info['center'][0]}, {marker_100_info['center'][1]})")
        data = struct.pack('ii', int(round(marker_100_info['center'][0])), int(round(marker_100_info['center'][1])))
        connection.sendall(data)
        
        # Find nearest ARUCO marker excluding ID 100
        nearest_aruco_id = min(
            (k for k in marker_info.keys() if k != 100),
            key=lambda k: np.linalg.norm(np.array(marker_info[k]['center']) - np.array(marker_100_info['center']))
        )

        # Calculate distance and rotation angle to the nearest marker
        if nearest_aruco_id in fixed_marker_coordinates:
            nearest_marker_coords = fixed_marker_coordinates[nearest_aruco_id]
            distance = np.linalg.norm(np.array(marker_100_info['center']) - np.array(nearest_marker_coords))
            x_diff = marker_100_info['corners'][0][0] - marker_100_info['corners'][1][0]
            y_diff = marker_100_info['corners'][0][1] - marker_100_info['corners'][1][1]
            rotation_angle = np.degrees(np.arctan2(y_diff, x_diff))
            rotation_angle = rotation_angle - 180

            # Write data to CSV file
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

    # Display frame with ARUCO marker detection
    cv2.namedWindow('GeoGuide', cv2.WINDOW_NORMAL)
    cv2.imshow('GeoGuide', frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()
