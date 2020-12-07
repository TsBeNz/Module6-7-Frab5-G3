import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time
import glob
from communication import communication
# aruco tracker

# import setting velue
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

# real world coordinate setup
cm_realworld_x = 400
cm_realworld_y = 400

# pixel coordinate setup
setup_pixel_x = 1000
setup_pixel_y = 1000

# point for Perspective Transform (x,y)
ax, ay = 562, 290
bx, by = 766, 284
cx, cy = 554, 479
dx, dy = 784, 478

# gobal variable
setup_point_count = 0


def tranform_to_realworld(input_x, input_y):
    print("eiei")


def mouse_click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("pixel coordinate: {}, {}".format(x, y))
        print("world coordinate: {}, {}, {} \n".format(
            x*(cm_realworld_x/setup_pixel_x), y*(cm_realworld_y/setup_pixel_y), 0))

def resize_percent(input_image ,percent):
    scale = (percent/100)
    width = int(input_image.shape[0] * scale)
    height = int(input_image.shape[1] * scale)
    dim = (height, width)
    return cv2.resize(input_image, dim, interpolation = cv2.INTER_AREA)

def read_multiple_image():
    real_path = '*.png'
    images = [cv2.imread(file) for file in glob.glob(real_path)]
    images2 = []
    for i in images:
        i = resize_percent(i, 60)
        images2.append(i)
    return images2


def Perspective(camara_source=0, setup_workspace=True , Path=[]):
    """
    perspective transformation
    ==========================

    camara_source 
    0 = camara
    1 = photo

    """
    # camera setting
    if camara_source == 0:
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        cap.set(3, 1280)
        cap.set(4, 720)

    elif camara_source == 1:
        frame = cv2.imread(
            'd:/3rd_year/module/Module6-7/High_Lv/moduleM1/output_pic/raw.jpg')

    if setup_workspace:
        print('eiei')
        # if camara_source == 0:
        #     ret, frame = cap.read()
        # set_realworld_frame(frame)
        # setup_world2(cap)

    cv2.namedWindow("tran")
    cv2.setMouseCallback("tran", mouse_click)
    j = 0
    while True:
        PIC.Move2point(Path[j][0], Path[j][1], Path[j][2], 0)
        print("eiei "+str(j))
        time.sleep(4)
        while True:
            if(j == len(Path)):
                    print("finish move!!")
                    break
            stage_eiei = 0
            if camara_source == 0:
                ret, frame = cap.read()
            # cv2.imshow("raw video",frame)

            elif camara_source == 1:
                frame = cv2.imread(
                    'd:/3rd_year/module/Module6-7/High_Lv/moduleM1/output_pic/raw.jpg')

            h,  w = frame.shape[:2]
            # print(str(h)+" "+str(w)+"\n")
            newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
                mtx, dist, (w, h), 1, (w, h))
            dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
            x, y, w, h = roi
            # cv2.imshow("undisbefore",dst)

            dst = dst[y:y+h, x:x+w]  # remap picture after undistort

            # plot point real frame
            gray = cv2.cvtColor(dst, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
            parameters = aruco.DetectorParameters_create() 
            corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                        parameters=parameters,
                                                                        cameraMatrix=mtx,
                                                                        distCoeff=dist)
            global ax,ay,bx,by,cx,cy,dx,dy
            if np.all(ids is not None):
                if (len(ids) == 4):
                    stage_eiei = 1
                    for i in range(0, len(ids)): 
                        eiei = corners[i].tolist()
                        if (int((ids.tolist())[i][0])) == 3:
                            ax,ay = eiei[0][0]
                        elif (int((ids.tolist())[i][0])) == 1:
                            bx,by = eiei[0][0]
                        elif (int((ids.tolist())[i][0])) == 2:
                            cx,cy = eiei[0][0]
                        elif (int((ids.tolist())[i][0])) == 0:
                            dx,dy = eiei[0][0]
                        rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02,mtx ,dist)
                        # (rvec - tvec).any()  # get rid of that nasty numpy value array error
                        # aruco.drawDetectedMarkers(dst, corners,ids)  # Draw A square around the markers
                        # aruco.drawAxis(dst, mtx ,dist, rvec, tvec, 0.01)  # Draw Axis
            # cv2.circle(dst, (int(ax), int(ay)), 1, (0, 0, 255), -1)
            # cv2.circle(dst, (int(bx), int(by)), 1, (0, 0, 255), -1)
            # cv2.circle(dst, (int(cx), int(cy)), 1, (0, 0, 255), -1)
            # cv2.circle(dst, (int(dx), int(dy)), 1, (0, 0, 255), -1)
            cv2.imshow("undis",dst)

            # Perspective Transform
            pts1 = np.float32([[ax, ay], [bx, by], [cx, cy], [dx, dy]])
            pts2 = np.float32([[0, 0], [setup_pixel_x, 0], [0, setup_pixel_y], [
                            setup_pixel_x, setup_pixel_y]])
            matrix = cv2.getPerspectiveTransform(pts1, pts2)
            result = cv2.warpPerspective(dst, matrix, (setup_pixel_x, setup_pixel_y))
            cv2.imshow("tran", result)
            if stage_eiei == 1:
                cv2.imwrite(str(j)+".png",result)
                time.sleep(3)
                j+=1
                break

            key = cv2.waitKey(20) & 0xFF
            if key == ord('q') or key == ord('ๆ'):
                cv2.destroyAllWindows()
                break
            elif key == ord('s') or key == ord('ฆ'):
                # cv2.imwrite("output_pic/raw.jpg",frame)
                # cv2.imwrite("output_pic/undis.jpg",dst)
                # cv2.imwrite("output_pic/tran.jpg",result)
                cv2.destroyAllWindows()
                break
        if(j == len(Path)):
            print("finish move!!")
            break

# def setup_world2(cap):
#     while True:
#         ret, frame = cap.read()
#         gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#         aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
#         parameters = aruco.DetectorParameters_create() 
#         corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
#                                                                     parameters=parameters,
#                                                                     cameraMatrix=mtx,
#                                                                     distCoeff=dist)
#         if np.all(ids is not None):
#             if (len(ids) == 4):
#                 for i in range(0, len(ids)): 
#                     eiei = corners[i].tolist()
#                     if (int((ids.tolist())[i][0])) == 4:
#                         ax,ay = eiei[0][0]
#                     elif (int((ids.tolist())[i][0])) == 1:
#                         bx,by = eiei[0][0]
#                     elif (int((ids.tolist())[i][0])) == 2:
#                         cx,cy = eiei[0][0]
#                     elif (int((ids.tolist())[i][0])) == 3:
#                         dx,dy = eiei[0][0]

#                     rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(corners[i], 0.02,mtx ,dist)
#                     (rvec - tvec).any()
#                     aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
#                     aruco.drawAxis(frame, mtx ,dist, rvec, tvec, 0.01)  # Draw Axis
#         key = cv2.waitKey(20) & 0xFF
#         if key == ord('q') or key == ord('ๆ'):
#             cv2.destroyAllWindows()
#             break

def set_realworld_frame(frame):
    
    global setup_point_count
    while True:
        cv2.imshow("set world", frame)
        cv2.setMouseCallback("set world", mouse_click_setup_world)
        key = cv2.waitKey(20) & 0xFF
        if setup_point_count == 4:
            break
        if key == ord('q') or key == ord('ๆ'):
            cv2.destroyAllWindows()
            break
    cv2.destroyAllWindows()


def mouse_click_setup_world(event, x, y, flags, param):
    global setup_point_count, ax, ay, bx, by, cx, cy, dx, dy
    if event == cv2.EVENT_RBUTTONDOWN:
        print("point : " + str(setup_point_count))
        # y-=19
        if setup_point_count == 0:
            ax, ay = x, y
        elif setup_point_count == 1:
            bx, by = x, y
        elif setup_point_count == 2:
            dx, dy = x, y
        elif setup_point_count == 3:
            cx, cy = x, y
        setup_point_count += 1


if __name__ == '__main__':
    try:
        PIC = communication()
        PIC.Go2home()
        inputtest = [[350,0,400,0],[350,60,400,0],[350,120,400,0],[350,180,400,0],[350,240,400,0],[350,300,400,0],[350,360,400,0],[350,420,400,0]]
        Perspective(Path= inputtest)
        images = read_multiple_image()
        set_of_image_to_stack = tuple(images)
        sequence = np.stack(set_of_image_to_stack, axis=3)
        result = np.median(sequence, axis=3).astype(np.uint8)
        cv2.imwrite("kuy.png",result)
        cv2.imshow("kuy world", result)
    except KeyboardInterrupt:
        print("\n\n\n\nShutdown ...\n\n\n\n")