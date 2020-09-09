import cv2
import numpy as np

# import setting velue
dist = np.load("setting/"+"dist.npy")
mtx = np.load("setting/"+"mtx.npy")
ret = np.load("setting/"+"ret.npy")
rvecs = np.load("setting/"+"rvecs.npy")
tvecs = np.load("setting/"+"tvecs.npy")

# camera setting
cap = cv2.VideoCapture(0,cv2.CAP_DSHOW)
cap.set(3,1280)
cap.set(4,720)

# point for Perspective Transform (x,y)
ax , ay = 562, 290   
bx , by = 766, 284
cx , cy = 554, 479
dx , dy = 784, 478

while True:
    ret, frame = cap.read()
    # cv2.imshow("raw video",frame)
    
    h,  w = frame.shape[:2]
    # print(str(h)+" "+str(w)+"\n")
    newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
    x, y, w, h = roi
    # cv2.imshow("undisbefore",dst)

    dst = dst[y:y+h, x:x+w] #remap picture after undistort
    
    # plot point real frame
    cv2.circle(dst, (ax , ay), 2, (0, 0, 255), -1)
    cv2.circle(dst, (bx , by), 2, (0, 0, 255), -1)
    cv2.circle(dst, (cx , cy), 2, (0, 0, 255), -1)
    cv2.circle(dst, (dx , dy), 2, (0, 0, 255), -1)
    cv2.imshow("undis",dst)

    # Perspective Transform
    pts1 = np.float32([[ax , ay], [bx , by], [cx , cy], [dx , dy]])
    pts2 = np.float32([[0, 0], [400, 0], [0, 400], [400, 400]])
    matrix = cv2.getPerspectiveTransform(pts1, pts2)
    result = cv2.warpPerspective(dst, matrix, (400, 400))
    cv2.imshow("tran",result)

    key = cv2.waitKey(20) & 0xFF
    if key == ord('q'):
        break
    elif key == ord('s'):
        cv2.imwrite("output_pic/raw.jpg",frame)
        cv2.imwrite("output_pic/undis.jpg",dst)
        cv2.imwrite("output_pic/tran.jpg",result)
        break