import cv2
import numpy as np
import os
import glob

# Defining the dimensions of checkerboard
CHECKERBOARD = (6,9 )
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Creating vector to store vectors of 3D points for each checkerboard image
objpoints = []
# Creating vector to store vectors of 2D points for each checkerboard image
imgpoints = [] 


# Defining the world coordinates for 3D points
objp = np.zeros((1, CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
prev_img_shape = None

cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
cap.set(3, 1280)
cap.set(4, 720)

# Extracting path of individual image stored in a given directory
# images = glob.glob('./images/*.jpg')
# images = glob.glob('cali_pic/*.jpg')
# for fname in images:
counter = 0
while True:
    # img = cv2.imread(fname)
    _, img = cap.read()
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    # If desired number of corners are found in the image then ret = true
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK + cv2.CALIB_CB_NORMALIZE_IMAGE)
    
    """
    If desired number of corner are detected,
    we refine the pixel coordinates and display 
    them on the images of checker board
    """
    if ret == True:
        objpoints.append(objp)
        # refining pixel coordinates for given 2d points.
        corners2 = cv2.cornerSubPix(gray, corners, (11,11),(-1,-1), criteria)
        
        imgpoints.append(corners2)

        # Draw and display the corners
        img = cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
    print(counter)
    counter += 1
    
    cv2.imshow('img',img)
    if counter == 20:
        break
    key = cv2.waitKey(500) & 0xFF
    if key == ord('q') or key == ord('ๆ'):
        cv2.destroyAllWindows()
        break

h,w = img.shape[:2]

"""
Performing camera calibration by 
passing the value of known 3D points (objpoints)
and corresponding pixel coordinates of the 
detected corners (imgpoints)
"""
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# f = open("setting.txt", "w")
# f.write("Woops! I have deleted the content!")
# f.close()

print("ret : ")
print(ret)
print("\nmtx : ")
print(mtx)
print("\ndist : ")
print(dist)
print("\nrvecs : ")
print(rvecs)
print("\ntvecs : ")
print(tvecs)

np.save("d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"ret.npy",ret)
np.save("d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"mtx.npy",mtx)
np.save("d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"dist.npy",dist)
np.save("d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"rvecs.npy",rvecs)
np.save("d:/3rd_year/module/Module6-7/High_Lv/moduleM1/setting/"+"tvecs.npy",tvecs)