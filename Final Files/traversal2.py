"""
* Team Id : GG_1949
* Author List : Ayush Karapagale, Akshit Gangawar, Prakhar Shukla, Sparsh Gautam
* Filename: traversal2.py
* Theme: Geo Guide -- EYRC
* Functions: get_ip_address, find_nearest_coordinate, load_coordinates
* Global Variables: sock, server_address, connection, client_address, aruco_dict, aruco_params, cap, mtx, dist, marker_info, lat_lon_data, fixed_marker_coordinates, csv_file_path, fieldnames, coordinates_dict
"""
#Variable Description
"""
1. s: Socket object used for network communication, specifically to determine the server's IP address in the get_ip_address function.
2. IP: String variable that stores the IP address of the server determined by the get_ip_address function.
3. sock: Socket object for establishing network communications, specifically for listening to incoming connections on a specified server address.
4. server_address: Tuple representing the server's address (IP and port) on which the sock object is bound and listens for incoming connections.
5. connection: Socket object representing the accepted connection from a client. It is used to communicate with the client by sending and receiving data.
6. client_address: Tuple containing the IP address and port number of the client that has connected to the server.
7. aruco_dict: Dictionary object for ARUCO marker detection, specifying the type of ARUCO markers to detect.
8. aruco_params: Object containing parameters for ARUCO marker detection, such as thresholds and algorithmic configurations.
9. cap: VideoCapture object used to interface with the video capturing hardware (e.g., webcam) for obtaining video frames.
10. mtx: Camera matrix used for correcting camera distortion in video frames.
11. dist: Coefficients for radial and tangential distortion correction of the camera.
12. marker_info: Dictionary used to store information about detected ARUCO markers in video frames, including their center coordinates and corner points.
13. lat_lon_data: DataFrame containing latitude and longitude data loaded from a CSV file, used for mapping ARUCO markers to geographic coordinates.
14. fixed_marker_coordinates: Dictionary mapping ARUCO marker IDs to their fixed geographic coordinates (latitude and longitude).
15. csv_file_path: String specifying the file path to a CSV file where data is to be written.
16. fieldnames: List of strings representing the column names for the CSV file where data is to be written.
17. coordinates_dict: Dictionary loaded with coordinates from a CSV file, mapping iterations to their corresponding x, y, latitude, and longitude values.
"""


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

"""
Function Name: get_ip_address
Input: None
Output: IP address of the server as a string.
Logic: Attempts to connect to a specific IP address to determine the server's IP. If unsuccessful, defaults to localhost.
Example Call: server_ip = get_ip_address()
"""

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

"""
Important: Configuration of ARUCO parameters directly influences the detection accuracy and performance.
"""

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
fieldnames = ['Latitude', 'Longitude','RotationAngle']



"""
Function Name: find_nearest_coordinate
Input: marker_center (tuple of x, y coordinates of the marker's center), coordinates_dict (dictionary with keys as iterations and values as tuples of x, y, lat, and long).
Output: The key (iteration number) of the nearest coordinate to the marker_center from the coordinates_dict.
Logic: Calculates the Euclidean distance between the marker center and each coordinate in the dictionary, returning the key of the minimum distance.
Example Call: nearest_iteration = find_nearest_coordinate((100, 200), coordinates_dict)
"""
def find_nearest_coordinate(marker_center, coordinates_dict):
    min_distance = float('inf')
    nearest_iteration = None
    for iteration, (x, y, _, _) in coordinates_dict.items():
        distance = np.linalg.norm(np.array(marker_center) - np.array((x, y)))
        if distance < min_distance:
            min_distance = distance
            nearest_iteration = iteration
    return nearest_iteration



"""
Function Name: load_coordinates
Input: filename (string representing the path to the CSV file containing coordinates).
Output: A dictionary with keys as iterations and values as tuples of x, y, latitude, and longitude.
Logic: Reads a CSV file and creates a dictionary mapping from iterations to their corresponding x, y, lat, and long values.
Example Call: coordinates_dict = load_coordinates('points.csv')
"""
def load_coordinates(filename):
    data = pd.read_csv(filename)
    coordinates_dict = {row['Iteration']: (row['X'], row['Y'], row['Lat'], row['Long']) for index, row in data.iterrows()}
    return coordinates_dict

#CSV file created by us for the QGIS navigation
coordinates_dict = load_coordinates('points.csv')

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
        nearest_iteration = find_nearest_coordinate(marker_100_info['center'], coordinates_dict)

        if nearest_iteration is not None:
            lat, lon = coordinates_dict[nearest_iteration][2], coordinates_dict[nearest_iteration][3]
            print(nearest_iteration)
            # Save the Lat, Lon in nearest3.csv
            with open('nearest3.csv', 'w', newline='') as csvfile:
                fieldnames = ['Latitude', 'Longitude']
                writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerow({'Latitude': lat, 'Longitude': lon})
            print(f"Saved nearest coordinates to nearest3.csv: Lat = {lat}, Lon = {lon}")

        print(f"Pixel Coordinates of Marker 100 Center: ({marker_100_info['center'][0]}, {marker_100_info['center'][1]})")
        data = struct.pack('ii', int(round(marker_100_info['center'][0])), int(round(marker_100_info['center'][1])))
        connection.sendall(data)
        

        x_diff = marker_100_info['corners'][0][0] - marker_100_info['corners'][1][0]
        y_diff = marker_100_info['corners'][0][1] - marker_100_info['corners'][1][1]

        # Write data to CSV file
        with open(csv_file_path, 'w', newline='') as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            if os.stat(csv_file_path).st_size == 0:  # Check if the file is empty
                writer.writeheader()
            writer.writerow({
                'Latitude': lat,
                'Longitude': lon,
            })

    # Display frame with ARUCO marker detection
    cv2.namedWindow('GeoGuide', cv2.WINDOW_NORMAL)
    cv2.imshow('GeoGuide', frame)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cap.release()
cv2.destroyAllWindows()