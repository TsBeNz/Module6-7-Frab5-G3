import cv2
import cv2.aruco as aruco
import numpy as np
import os

# https://medium.com/@aliyasineser/aruco-marker-tracking-with-opencv-8cb844c26628

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3, 1280)
cap.set(4, 720)

dist = np.load(
    "d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"dist.npy")
mtx = np.load(
    "d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"mtx.npy")
ret = np.load(
    "d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"ret.npy")
rvecs = np.load(
    "d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"rvecs.npy")
tvecs = np.load(
    "d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"tvecs.npy")


while True:
    ret, frame = cap.read()
    # frame = cv2.imread('D:/3rd_year/module/Module6-7/High_Lv/moduleM1/aruco.jpg')
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
    # gray = cv2.imread('D:/3rd_year/module/Module6-7/High_Lv/moduleM1/aruco.jpg',0)
    aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)  # Use 5x5 dictionary to find markers
    parameters = aruco.DetectorParameters_create()  # Marker detection parameters
# lists of ids and the corners beloning to each id
    corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=mtx,
                                                                distCoeff=dist)

    if np.all(ids is not None):  # If there are markers found by detector
        # print(ids.tolist())
        for i in range(0, len(ids)):  # Iterate in markers
            # Estimate pose of each marker and return the values rvec and tvec---different from camera coefficients
            eiei = corners[i].tolist()
            # print(eiei)
            x,y = eiei[0][0]
            cv2.circle(frame, (int(x), int(y)), 2, (0, 0, 255), -1)
            rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02,mtx ,dist)
            (rvec - tvec).any()  # get rid of that nasty numpy value array error
            aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
            aruco.drawAxis(frame, mtx ,dist, rvec, tvec, 0.01)  # Draw Axis
    # print("\n\n\n")
# Display the resulting frame
    cv2.imshow('frame', frame)
    # Wait 3 milisecoonds for an interaction. Check the key and do the corresponding job.
    key = cv2.waitKey(20) & 0xFF
    if key == ord('q'):  # Quit
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()