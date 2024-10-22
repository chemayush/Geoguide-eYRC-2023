'''
*****************************************************************************************
*
*        		===============================================
*           		Geo Guide (GG) Theme (eYRC 2023-24)
*        		===============================================
*
*  This script is to implement Task 4A of Geo Guide (GG) Theme (eYRC 2023-24).
*  
*  This software is made available on an "AS IS WHERE IS BASIS".
*  Licensee/end user indemnifies and will keep e-Yantra indemnified from
*  any and all claim(s) that emanate from the use of the Software or 
*  breach of the terms of this agreement.
*
*****************************************************************************************
'''

# Team ID:			[ GG 1949 ]
# Author List:		[ Ayush Karapagale, Akshit Gangwar, Prakhar Shukla, Sparsh Gautam ]
# Filename:			task_4a.py


####################### IMPORT MODULES #######################
import cv2
import math
from ultralytics import YOLO
import os  
##############################################################



################# ADD UTILITY FUNCTIONS HERE #################

"""
You are allowed to add any number of functions to this code.
"""

def get_dict(coords):
    if len(coords) < 5: 
        return {'A': '', 'B': '', 'C': '', 'D': '', 'E': ''}
    
    order = sorted(coords, key=lambda x: x[2])
    if order[2][1] > order[1][1]:
        return {'A': str(order[4][0]), 'B': str(order[3][0]), 'C': str(order[2][0]), 'D': str(order[1][0]), 'E': str(order[0][0])}
    else:
        return {'A': str(order[4][0]), 'B': str(order[3][0]), 'C': str(order[1][0]), 'D': str(order[2][0]), 'E': str(order[0][0])}


##############################################################


def task_4a_return():
    """
    Purpose:
    ---
    Only for returning the final dictionary variable
    
    Arguments:
    ---
    You are not allowed to define any input arguments for this function. You can 
    return the dictionary from a user-defined function and just call the 
    function here

    Returns:
    ---
    `identified_labels` : { dictionary }
        dictionary containing the labels of the events detected
    """  
    identified_labels = {}  
    
##############	ADD YOUR CODE HERE	##############
    identified_labels = {}  

    ##############    ADD YOUR CODE HERE    ##############
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    cap.set(3, 1920)
    cap.set(4, 1080)

    model = YOLO("best8m.pt")
    classNames = ['combat', 'destroyed_buildings', 'fire', 'military_vehicles', 'human_aid_rehabilitation']

    x, y, width, height = 305, 0, 1265, 1080

    while True:
        success, img = cap.read()
        cropped_img = img[y:y + height, x:x + width]

        results = model(cropped_img, agnostic_nms=True, verbose=False)
        coordinates = []

        for r in results:
            boxes = r.boxes

            for box in boxes:
                # bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                confidence = math.ceil((box.conf[0] * 100)) / 100

                label = ''

                if confidence > 0.6:
                    cls = int(box.cls[0])
                    label = classNames[cls]
                    
                    cv2.rectangle(cropped_img, (x1, y1), (x2, y2), (0, 255, 0), 3)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    color = (0, 255, 0)
                    thickness = 2

                    cv2.putText(cropped_img, label, (x1, y1-10), font, 0.6, color, thickness)
                    
                coordinates.append((label, x1, y1))

        identified_labels = get_dict(coordinates)
        print(identified_labels)
        cv2.namedWindow('Webcam', cv2.WINDOW_NORMAL)
        cv2.imshow('Webcam', cropped_img)
        if cv2.waitKey(1) == ord('q'):
            break
        
    cap.release()
    cv2.destroyAllWindows()
    ##################################################
    return identified_labels
##################################################


###############	Main Function	#################
if __name__ == "__main__":
    identified_labels = task_4a_return()
    print(identified_labels)