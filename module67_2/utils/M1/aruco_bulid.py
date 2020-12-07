import cv2
import numpy  as np
import cv2.aruco as aruco

# pip install opencv-contrib-python

# Load the predefined dictionary
dictionary = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

# Generate the marker
markerImage = np.zeros((200, 200), dtype=np.uint8)
markerImage = aruco.drawMarker(dictionary, 3, 200, markerImage, 1)

cv2.imwrite("marker.png", markerImage)

