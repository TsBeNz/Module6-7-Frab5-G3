import os
import numpy as np
import cv2
import math
import statistics
# from scipy.optimize import curve_fit
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
# from get_map import get_map

def get_points(img, color):
    points = []
    for y in range(0, img.shape[0]):
        for x in range(0, img.shape[1]):
            if img[y,x] == color:
                points.append([y,x])

    return points

def get_distance(p1, p2):
    return math.sqrt(((p2[0] - p1[0])**2) + ((p2[1]-p1[1])**2))

def get_angle(p1, p2):
    return math.degrees(math.atan2((p2[0]-p1[0]),(p2[1] - p1[1])))

def check_line(p1, p2, black_points, w, h):
    black_img = np.zeros((w, h), dtype = "uint8")
    black_img1 = cv2.line(black_img.copy(), (int(p1[1]),int(p1[0])), (int(p2[1]),int(p2[0])), (255,255,255), 20)
    # plt.figure()
    # plt.imshow(black_img, cmap='gray')
    # plt.show()
    
    sub_white_points = get_points(black_img1, 255)
    for i in sub_white_points:
        if i in black_points:
            return False
        # black_img20 = cv2.line(black_img.copy(), (int(p1[1]),int(p1[0])), (int(p2[1]),int(p2[0])), (255,255,255), 20)
        # sub_white_points2 = get_points(black_img20,255)
        # for j in sub_white_points2:
        #     if j in black_points:
        #         return False
    return True

def check_angle(white_points, p1, r):
    angles = []
    black_img = np.zeros((400, 400), dtype = "uint8")
    cv2.circle(black_img, (int(p1[1]), int(p1[0])), r, 255, 1)
    # plt.figure()
    # plt.imshow(black_img, cmap='gray')
    # plt.show()
    sub_white_points = get_points(black_img, 255)
    for i in sub_white_points:
        if i in white_points:
            angles.append(get_angle(p1,i))
    return angles 
    
def find_next_2d_node(current, end, nodes, sk_white_points, white_points, black_points):
    if check_line(current, end, black_points, 400, 400):
        nodes.append([end[0], end[1]])
        return nodes
   
    angles = check_angle(sk_white_points, current, 20)
    new_node = []
    for i in sk_white_points:
        i_angle = get_angle(current, i)
        for a in angles:
            if abs(i_angle - a) < 5:
                i_dist = get_distance(current,i)
                new_node.append([i[0], i[1], i_dist, i_angle])

    
    new_node = sorted(new_node, key=lambda x: x[2], reverse=True)
    print(nodes)
    for i in new_node:
        if check_line(current, [i[0], i[1]], black_points, 400, 400):
            nodes.append([i[0], i[1], i[3]])
            return nodes

def find_all_z(current, end, hsv_img, no_sign_area):
    black_img = np.zeros((400, 400), dtype = "uint8")
    cv2.line(black_img, (int(current[1]),int(current[0])), (int(end[1]), int(end[0])), (255,255,255), 1)
    black_img = cv2.bitwise_and(black_img,black_img, mask=no_sign_area)
    white_points = get_points(black_img, 255)
    white_points = sorted(white_points, key=lambda x: get_distance([current[0],current[1]], [x[0], x[1]]))

    z_list = []
    for i in white_points:
        z = hsv_img[i[0],i[1],1]
        z_list.append(z)

    return z_list, white_points


def find_next_3d_node(current, end, new_z, white_points, i):
    node_3d = []
    node_3d.append([white_points[i][5][0], white_points[i][5][1], new_z[i][5]])
    node_3d.append([white_points[i][-5][0], white_points[i][-5][1], new_z[i][-5]])
    node_3d.append([end[0], end[1], new_z[i][-5]])
    return node_3d

def find_xy(sk_img, area_img, sign_pos):
    """
    param: 
    sk_img = skeleton image
    area_img = avaliable area represented by white color
    sign_pos = coordinate of sign sorted by order
    
    return: list of coordinate [x y] sorted by order 
            path_img (white contour on black img)
    """
    nodes = [[sign_pos[0][2], sign_pos[0][1], 0]]
    end = [sign_pos[-1][2], sign_pos[-1][1]]
    sk_white_points = get_points(sk_img.copy(), 255)
    white_points = get_points(area_img.copy(), 255)
    black_points = get_points(area_img.copy(), 0)
    while nodes[-1][0] != end[0] and  nodes[-1][1] != end[1]:
        nodes = find_next_2d_node(nodes[-1], end, nodes, sk_white_points, white_points, black_points)
        print(nodes)
    return nodes

def find_z(hsv_img, nodes, no_sign_area):
    """
    param: 
    hsv_img = hsv image
    path image = image recieve from find_xy

    skeketonize path image 
    find s value 
    find position x y of the peak s 
    find angle via arctan

    return: list of coordinate [x y z]
    """

    all_z = []
    all_z_list = []
    all_white_points = []
    for i in range(len(nodes) - 1):
        z_list, white_points = find_all_z(nodes[i], nodes[i+1], hsv_img.copy(), no_sign_area.copy())
        all_z_list.append(z_list)
        all_white_points.append(white_points)
        for j in z_list:
            all_z.append(j)
    minz, maxz = min(all_z), max(all_z)
    # b = np.array(all_z_list)
    # print(b.shape)
    new_all_z_list = []
    for i in all_z_list:
        lis = []
        for j in i:
            lis.append((100 * ((j - minz)/(maxz-minz)))+155)
        new_all_z_list.append(lis)
    # a = np.array(new_all_z_list)
    # print(a.shape)
    # print(all_z)
    # print(all_z_list)
    # print(all_white_points)

    nodes_3d = []
    for i in range(len(nodes) - 1):
        node_3d = find_next_3d_node(nodes[i], nodes[i+1], new_all_z_list, all_white_points, i)
        for j in node_3d:
            nodes_3d.append(j)
    nodes_3d.insert(0, [nodes[0][0], nodes[0][1], nodes_3d[0][2]])
    return nodes_3d

def find_angle(pos):
    """
    pos = list of coordinate [x y z] from find_z
    find arctan y/x
    return list of command [x y z z-angle]
    """
    for i in range(len(pos) - 1):
        angle = get_angle([pos[i][0], pos[i][1]], [pos[i+1][0], pos[i+1][1]])
        pos[i].append(angle+90)
        if pos[i+1] == pos[-1]:
            pos[i+1].append(angle)
    return pos

def get_path(hsv_img, area_img, no_sign_img, sk_img, sign_pos):
    # print(sign_pos)
    # plt.figure()
    # plt.imshow(no_sign_img, cmap='gray')
    # plt.show()
    xy = find_xy(sk_img, area_img, sign_pos)
    print(xy)
    # xy = [[319.0, 341.0, 0], [319, 227, 180.0], [70.5, 77.0]]
    xyz = find_z(hsv_img, xy, no_sign_img)
    print(xyz)
    pos_list = find_angle(xyz)
    print(pos_list)
    return pos_list
    # send_pic(pos_list)
# hsv_img, area_img, no_sign_img, sk_img, sign_pos = get_map()
# get_path(hsv_img, area_img, no_sign_img, sk_img, sign_pos)