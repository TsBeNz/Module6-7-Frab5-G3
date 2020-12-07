import cv2
import cv2.aruco as aruco
import numpy as np
import os

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
cm_realworld_x = 24.6
cm_realworld_y = 16

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


def find_mid_of_aruco(list_point=[]):
    ax = 0
    bx = 0
    ay = 0
    by = 0
    ax, ay = list_point[0][0]
    bx, by = list_point[0][2]
    return (ax+bx)//2, (ay+by)//2

#((x, y), (x2, y2))


def line_intersection(line1=[[]], line2=[[]]):
    xdiff = (line1[0][0] - line1[1][0], line2[0][0] - line2[1][0])
    ydiff = (line1[0][1] - line1[1][1], line2[0][1] - line2[1][1])

    def det(a, b):
        return a[0] * b[1] - a[1] * b[0]
    div = det(xdiff, ydiff)
    if div == 0:
        return -1, -1
        # raise Exception('lines do not intersect')
    d = (det(*line1), det(*line2))
    x = det(d, xdiff) / div
    y = det(d, ydiff) / div
    return int(x), int(y)


def find_conner(corners, ids, side_a=[], side_b=[]):
    output_a, output_b = find_mid_of_aruco(
        corners[ids.tolist().index([min(side_a)])].tolist())
    output_c, output_d = find_mid_of_aruco(
        corners[ids.tolist().index([max(side_a)])].tolist())
    output_e, output_f = find_mid_of_aruco(
        corners[ids.tolist().index([min(side_b)])].tolist())
    output_g, output_h = find_mid_of_aruco(
        corners[ids.tolist().index([max(side_b)])].tolist())
    buffer1, buffer2 = line_intersection([[output_a, output_b], [output_c, output_d]], [
                                         [output_e, output_f], [output_g, output_h]])
    return buffer1, buffer2

def Perspective(camara_source=0, setup_workspace=True):
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
    while True:
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
        global ax, ay, bx, by, cx, cy, dx, dy
        if np.all(ids is not None):
            top_side = [12, 11, 10, 9, 8]
            top_side_real = []
            right_side = [4, 5, 6, 7, 8]
            right_side_real = []
            left_side = [12, 13, 14, 15, 16]
            left_side_real = []
            bottom_side = [16, 1, 2, 3, 4]
            bottom_side_real = []
            for i in range(len(ids)):
                if ids[i][0] in top_side:
                    top_side_real.append(ids[i][0])
                elif ids[i][0] in right_side:
                    right_side_real.append(ids[i][0])
                elif ids[i][0] in left_side:
                    left_side_real.append(ids[i][0])
                elif ids[i][0] in bottom_side:
                    bottom_side_real.append(ids[i][0])

            if ((len(top_side_real) > 0) and (len(left_side_real) > 0)):
                if 12 in ids:
                    ax, ay = find_mid_of_aruco(
                        corners[ids.tolist().index([12])].tolist())
                else:
                    buffer1, buffer2 = find_conner(
                        corners, ids, top_side_real, left_side_real)
                    if (buffer1 != -1 and buffer2 != -1):
                        ax, ay = buffer1, buffer2

            if ((len(top_side_real) > 0) and (len(right_side_real) > 0)):
                if 8 in ids:
                    bx, by = find_mid_of_aruco(
                        corners[ids.tolist().index([8])].tolist())
                else:
                    buffer1, buffer2 = find_conner(
                        corners, ids, top_side_real, right_side_real)
                    if (buffer1 != -1 and buffer2 != -1):
                        bx, by = buffer1, buffer2

            if ((len(left_side_real) > 0) and (len(bottom_side_real) > 0)):
                if 16 in ids:
                    cx, cy = find_mid_of_aruco(
                        corners[ids.tolist().index([16])].tolist())
                else:
                    buffer1, buffer2 = find_conner(
                        corners, ids, bottom_side_real, left_side_real)
                    if (buffer1 != -1 and buffer2 != -1):
                        cx, cy = buffer1, buffer2

            if ((len(right_side_real) > 0) and (len(bottom_side_real) > 0)):
                if 4 in ids:
                    dx, dy = find_mid_of_aruco(
                        corners[ids.tolist().index([4])].tolist())
                else:
                    buffer1, buffer2 = find_conner(
                        corners, ids, bottom_side_real, right_side_real)
                    if (buffer1 != -1 and buffer2 != -1):
                        dx, dy = buffer1, buffer2

        # (rvec - tvec).any()  # get rid of that nasty numpy value array error
        # rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(
        #     corners, 0.02, mtx, dist)
        # Draw A square around the markers
        aruco.drawDetectedMarkers(dst, corners,ids)
        # aruco.drawAxis(dst, mtx ,dist, rvec, tvec, 0.01)  # Draw Axis
        cv2.circle(dst, (int(ax), int(ay)), 3, (0, 255, 255), -1)
        cv2.circle(dst, (int(bx), int(by)), 3, (0, 255, 255), -1)
        cv2.circle(dst, (int(cx), int(cy)), 3, (0, 255, 255), -1)
        cv2.circle(dst, (int(dx), int(dy)), 3, (0, 255, 255), -1)
        cv2.imshow("undis", dst)

        # Perspective Transform
        pts1 = np.float32([[ax, ay], [bx, by], [cx, cy], [dx, dy]])
        pts2 = np.float32([[0, 0], [setup_pixel_x, 0], [0, setup_pixel_y], [
                          setup_pixel_x, setup_pixel_y]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(
            dst, matrix, (setup_pixel_x, setup_pixel_y))
        cv2.imshow("tran", result)

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
        Perspective()
    except KeyboardInterrupt:
        print("\n\n\n\nShutdown ...\n\n\n\n")
