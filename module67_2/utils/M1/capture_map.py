import cv2 as cv2
import cv2.aruco as aruco
import numpy as np
import os
import time
import glob
# from communication import communication

PATH = os.path.dirname(os.path.abspath(__file__))
# import setting velue
dist = np.load(PATH + "/setting/dist.npy")
mtx = np.load(PATH + "/setting/mtx.npy")
ret = np.load(PATH + "/setting/ret.npy")
rvecs = np.load(PATH + "/setting/rvecs.npy")
tvecs = np.load(PATH + "/setting/tvecs.npy")

setup_pixel_x = 1000
setup_pixel_y = 1000

ax, ay = 562, 290
bx, by = 766, 284
cx, cy = 554, 479
dx, dy = 784, 478


pic_offset = 35  ##pixel


def find_mid_of_aruco(list_point=[]):
    """
    For Find Mid of aruco tag
    =========================
    """
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


def Perspective(debug=0, index_pic=0, cap=cv2.VideoCapture(0, cv2.CAP_DSHOW)):
    """
    perspective transformation
    ==========================
    """
    count = 0
    while True:
        state_chack = 0
        ret, frame = cap.read()

        # h,  w = frame.shape[:2]
        # newcameramtx, roi = cv2.getOptimalNewCameraMatrix(
        #     mtx, dist, (w, h), 1, (w, h))
        # dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        # x, y, w, h = roi
        # dst = dst[y:y+h, x:x+w]  # resize picture after undistort

        # aruco
        # change undis or dis in this line
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        aruco_dict = aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters_create()
        corners, ids, rejected_img_points = aruco.detectMarkers(gray, aruco_dict,
                                                                parameters=parameters,
                                                                cameraMatrix=mtx,
                                                                distCoeff=dist)
        global ax, ay, bx, by, cx, cy, dx, dy
        if np.all(ids is not None):
            conner_list = [16,8,24,32]
            top_side = [16,15,14,13,12,11,10,9,8]
            top_side_real = []
            right_side = [8,7,6,5,4,3,2,1,32]
            right_side_real = []
            left_side = [16,17,18,19,20,21,22,23,24]
            left_side_real = []
            bottom_side = [24,25,26,27,28,29,30,31,32]
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
                if conner_list[0] in ids:
                    state_chack += 1
                    ax, ay = find_mid_of_aruco(
                        corners[ids.tolist().index([conner_list[0]])].tolist())
                else:
                    buffer1, buffer2 = find_conner(
                        corners, ids, top_side_real, left_side_real)
                    if (buffer1 != -1 and buffer2 != -1):
                        ax, ay = buffer1, buffer2
                        state_chack += 1

            if ((len(top_side_real) > 0) and (len(right_side_real) > 0)):
                if conner_list[1] in ids:
                    state_chack += 1
                    bx, by = find_mid_of_aruco(
                        corners[ids.tolist().index([conner_list[1]])].tolist())
                else:
                    buffer1, buffer2 = find_conner(
                        corners, ids, top_side_real, right_side_real)
                    if (buffer1 != -1 and buffer2 != -1):
                        bx, by = buffer1, buffer2
                        state_chack += 1

            if ((len(left_side_real) > 0) and (len(bottom_side_real) > 0)):
                if conner_list[2] in ids:
                    state_chack += 1
                    cx, cy = find_mid_of_aruco(
                        corners[ids.tolist().index([conner_list[2]])].tolist())
                else:
                    buffer1, buffer2 = find_conner(
                        corners, ids, bottom_side_real, left_side_real)
                    if (buffer1 != -1 and buffer2 != -1):
                        cx, cy = buffer1, buffer2
                        state_chack += 1

            if ((len(right_side_real) > 0) and (len(bottom_side_real) > 0)):
                if conner_list[3] in ids:
                    state_chack += 1
                    dx, dy = find_mid_of_aruco(
                        corners[ids.tolist().index([conner_list[3]])].tolist())
                else:
                    buffer1, buffer2 = find_conner(
                        corners, ids, bottom_side_real, right_side_real)
                    if (buffer1 != -1 and buffer2 != -1):
                        dx, dy = buffer1, buffer2
                        state_chack += 1

        if(state_chack == 4):
            cv2.circle(frame, (int(ax), int(ay)), 3, (0, 255, 255), -1)
            cv2.circle(frame, (int(bx), int(by)), 3, (0, 255, 255), -1)
            cv2.circle(frame, (int(cx), int(cy)), 3, (0, 255, 255), -1)
            cv2.circle(frame, (int(dx), int(dy)), 3, (0, 255, 255), -1)
            ax, ay = ax+pic_offset, ay+pic_offset
            bx, by = bx-pic_offset, by+pic_offset
            cx, cy = cx+pic_offset, cy-pic_offset
            dx, dy = dx-pic_offset, dy-pic_offset
            count += 1    
        # (rvec - tvec).any()  # get rid of that nasty numpy value array error
        # rvec, tvec, markerPoints = aruco.estimatePoseSingleMarkers(
        #     corners, 0.02, mtx, dist)
        # Draw A square around the markers
        aruco.drawDetectedMarkers(frame, corners)
        # aruco.drawAxis(frame, mtx ,dist, rvec, tvec, 0.01)  # Draw Axis
        cv2.circle(frame, (int(ax), int(ay)), 3, (255, 255, 255), -1)
        cv2.circle(frame, (int(bx), int(by)), 3, (255, 255, 255), -1)
        cv2.circle(frame, (int(cx), int(cy)), 3, (255, 255, 255), -1)
        cv2.circle(frame, (int(dx), int(dy)), 3, (255, 255, 255), -1)
        cv2.imshow("undis", frame)

        # Perspective Transform
        pts1 = np.float32([[ax, ay], [bx, by], [cx, cy], [dx, dy]])
        pts2 = np.float32([[0, 0], [setup_pixel_x, 0], [0, setup_pixel_y], [
            setup_pixel_x, setup_pixel_y]])
        matrix = cv2.getPerspectiveTransform(pts1, pts2)
        result = cv2.warpPerspective(
            frame, matrix, (setup_pixel_x, setup_pixel_y))
        cv2.imshow("tran", result)
        
        key = cv2.waitKey(20) & 0xFF
        if count >= 5:
            print("capture Finish!!")
            cv2.imwrite(PATH + "/pic_input/"+str(index_pic)+".png", result)
            return result
        if key == ord('q') or key == ord('ๆ'):
            cv2.destroyAllWindows()
            break
        elif key == ord('s') or key == ord('ฆ'):
            cv2.destroyAllWindows()
            break

def crop_sign(communication,count):
    try:
        cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
        cap.set(3, 1280)
        cap.set(4, 720)
        cap.set(cv2.CAP_PROP_AUTOFOCUS,0) # turn the autofocus off
        Square_Root = communication(port="com7", baudrate=500000)
        Square_Root.Offset(offsetxy=0, offsetz=0)
        Square_Root.Go2home()
        Square_Root.Velocity_max(80)
        Path = [[350, 230, 400, 0]]
        i = 0
        out_images = []
        while True:
            if(i == len(Path)):
                print("finish move!!")
                break
            print("Move to " + str(Path[i]))
            Square_Root.Move2point(Path[i][0], Path[i][1], Path[i][2], 0)
            time.sleep(0.1)
            while True:
                if(int(Square_Root.Status_point()[4]) == 1):
                    break
                time.sleep(0.1)
            print("Move success!!")
            i += 1
            time.sleep(1)
            out_images = Perspective(index_pic=i, cap=cap)
        cv2.destroyAllWindows()
        cv2.imshow("output world", out_images)
        out_images = cv2.resize(out_images, (400,400))
        out_images = out_images[0:100, 0:100]
        cv2.imwrite("D:/module67_2/utils/imgs/raw_templates/{}.png".format(count), out_images)
        Square_Root.Move2point(0, 0, 400, 0)
        # while True:
        #     key = cv2.waitKey(20) & 0xFF
        #     if key == 27:
        #         cv2.destroyAllWindows()
        #         break
        #     elif key == ord('q') or key == ord('ๆ'):
        #         cv2.destroyAllWindows()
        #         break

    except KeyboardInterrupt:
        print("\n\n\n\nShutdown ...\n\n\n\n")

def capture_map(communication):
        try:
            cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
            cap.set(3, 1280)
            cap.set(4, 720)
            cap.set(cv2.CAP_PROP_AUTOFOCUS,0) # turn the autofocus off
            Square_Root = communication(port="com7", baudrate=500000)
            Square_Root.Offset(offsetxy=0, offsetz=0)
            Square_Root.Go2home()
            Square_Root.Velocity_max(80)
            Path = [[350, 0, 400, 0], [350, 40, 400, 0], [350, 80, 400, 0], [350, 120, 400, 0], [350, 160, 400, 0],
                    [350, 200, 400, 0], [350, 240, 400, 0], [350, 280, 400, 0], [350, 320, 400, 0], [350, 360, 400, 0], [350, 400, 400, 0]]
            # Path = [[350, 0, 400, 0], [350, 80, 400, 0], [350, 160, 400, 0], [350, 240, 400, 0], [350, 320, 400, 0],
            #         [350, 400, 400, 0]]
            i = 0
            images = []
            while True:
                if(i == len(Path)):
                    print("finish move!!")
                    break
                print("Move to " + str(Path[i]))
                Square_Root.Move2point(Path[i][0], Path[i][1], Path[i][2], 0)
                time.sleep(0.1)
                while True:
                    if(int(Square_Root.Status_point()[4]) == 1):
                        break
                    time.sleep(0.1)
                print("Move success!!")
                i += 1
                images.append(Perspective(index_pic=i, cap=cap))
                print("Picture : " + str(i))
                time.sleep(2)
            sequence = np.stack(tuple(images), axis=3)
            result = np.median(sequence, axis=3).astype(np.uint8)
            time_out = 0
            cv2.destroyAllWindows()
            result = cv2.resize(result, (400,400))
            cv2.imwrite("D:/module67_2/utils/imgs/maps/output.png", result)
            cv2.imshow("output world", result)
            Square_Root.Move2point(0, 0, 400, 0)
            while True:
                time_out += 1
                key = cv2.waitKey(20) & 0xFF
                if time_out >= 500:
                    cv2.destroyAllWindows()
                    break
                if key == 27:
                    cv2.destroyAllWindows()
                    break
                elif key == ord('q') or key == ord('ๆ'):
                    cv2.destroyAllWindows()
                    break

        except KeyboardInterrupt:
            print("\n\n\n\nShutdown ...\n\n\n\n")

# if __name__ == '__main__':
#     try:
#         print("a")
#         cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
#         cap.set(3, 1280)
#         cap.set(4, 720)
#         cap.set(cv2.CAP_PROP_AUTOFOCUS,0) # turn the autofocus 
#         print("b")
#         Square_Root = communication(port="com7", baudrate=500000)
#         print("c")
#         Square_Root.Offset(offsetxy=0, offsetz=0)
#         Square_Root.Go2home()
#         Square_Root.Velocity_max(80)
#         Path = [[350, 230, 400, 0]]
#         i = 0
#         out_images = []
#         while True:
#             if(i == len(Path)):
#                 print("finish move!!")
#                 break
#             print("Move to " + str(Path[i]))
#             Square_Root.Move2point(Path[i][0], Path[i][1], Path[i][2], 0)
#             time.sleep(0.1)
#             while True:
#                 if(int(Square_Root.Status_point()[4]) == 1):
#                     break
#                 time.sleep(0.1)
#             print("Move success!!")
#             i += 1
#             time.sleep(1)
#             out_images = Perspective(index_pic=i, cap=cap)
#         cv2.destroyAllWindows()
#         cv2.imshow("output world", out_images)
#         Square_Root.Move2point(0, 0, 400, 0)
#         while True:
#             key = cv2.waitKey(20) & 0xFF
#             if key == 27:
#                 cv2.destroyAllWindows()
#                 break
#             elif key == ord('q') or key == ord('ๆ'):
#                 cv2.destroyAllWindows()
#                 break

#     except KeyboardInterrupt:
#         print("\n\n\n\nShutdown ...\n\n\n\n")
#     try:
#         cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
#         cap.set(3, 1280)
#         cap.set(4, 720)
#         cap.set(cv2.CAP_PROP_AUTOFOCUS,0) # turn the autofocus off
#         Square_Root = communication(port="com7", baudrate=500000)
#         Square_Root.Offset(offsetxy=0, offsetz=0)
#         Square_Root.Go2home()
#         Square_Root.Velocity_max(80)
#         Path = [[350, 0, 400, 0], [350, 40, 400, 0], [350, 80, 400, 0], [350, 120, 400, 0], [350, 160, 400, 0],
#                 [350, 200, 400, 0], [350, 240, 400, 0], [350, 280, 400, 0], [350, 320, 400, 0], [350, 360, 400, 0], [350, 400, 400, 0]]
#         # Path = [[350, 0, 400, 0], [350, 80, 400, 0], [350, 160, 400, 0], [350, 240, 400, 0], [350, 320, 400, 0],
#         #         [350, 400, 400, 0]]
#         i = 0
#         images = []
#         while True:
#             if(i == len(Path)):
#                 print("finish move!!")
#                 break
#             print("Move to " + str(Path[i]))
#             Square_Root.Move2point(Path[i][0], Path[i][1], Path[i][2], 0)
#             time.sleep(0.1)
#             while True:
#                 if(int(Square_Root.Status_point()[4]) == 1):
#                     break
#                 time.sleep(0.1)
#             print("Move success!!")
#             i += 1
#             images.append(Perspective(index_pic=i, cap=cap))
#             print("Picture : " + str(i))
#             time.sleep(2)
#         sequence = np.stack(tuple(images), axis=3)
#         result = np.median(sequence, axis=3).astype(np.uint8)
#         time_out = 0
#         cv2.destroyAllWindows()
#         cv2.imwrite("D:/module67_2/utils/imgs/maps/output.png", result)
#         cv2.imshow("output world", result)
#         Square_Root.Move2point(0, 0, 400, 0)
#         while True:
#             time_out += 1
#             key = cv2.waitKey(20) & 0xFF
#             if time_out >= 500:
#                 cv2.destroyAllWindows()
#                 break
#             if key == 27:
#                 cv2.destroyAllWindows()
#                 break
#             elif key == ord('q') or key == ord('ๆ'):
#                 cv2.destroyAllWindows()
#                 break
#         # Square_Root.Offset(offsetxy=20, offsetz=0)
#         # Square_Root.Griping_Rod(point = 0)
#         # Square_Root.Velocity_max(80)
#         # # Square_Root.Go2home()
#         # # Square_Root.Manual_Control()
#         # inputtest = [[55, 320, 400, 0], [55, 320, 250, 0], [192, 308, 250, 0], [
#         #     315, 98, 150, 60]]


#         # # [[55.0, 320.5, 0], [192, 308, 95.21328240475526], [315.0, 98.0, 0]]
#         # Square_Root.Path_list(inputtest)

#     except KeyboardInterrupt:
#         print("\n\n\n\nShutdown ...\n\n\n\n")
