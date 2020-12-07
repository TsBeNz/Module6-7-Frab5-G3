import os
import cv2
import imutils
import numpy as np
from matplotlib import pyplot as plt
from skimage.morphology import skeletonize
from skimage.exposure import match_histograms

PATH = os.path.dirname(os.path.abspath(__file__))
RAW_TEMPLATE_PATH = PATH + '/imgs/raw_templates'
NUM_TEMPLATES = len([f for f in os.listdir(RAW_TEMPLATE_PATH) if os.path.isfile(os.path.join(RAW_TEMPLATE_PATH, f))])

MAP_IMG = cv2.imread(PATH + '/imgs/maps/output.png')
# MAP_IMG = cv2.resize(MAP_IMG, (400,400))
# cv2.imwrite(PATH + '/imgs/maps/output.png', MAP_IMG)
MAP_IMG_GRAY = cv2.cvtColor(MAP_IMG.copy(), cv2.COLOR_BGR2GRAY)

def preprocessing(img):
    blur_img = cv2.GaussianBlur(img, (5,5), cv2.BORDER_CONSTANT)
    unsharpen_img = cv2.addWeighted(img, 1.8, blur_img, -0.8, 0, img)
    return unsharpen_img

def drawContours(img, thickness, min_area=0):
    contours,_ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    black_img = np.zeros((400, 400), dtype = "uint8")
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > min_area and area < 140000:
            cv2.drawContours(img, [cnt], -1, 255, thickness=thickness)
            cv2.drawContours(black_img, [cnt], -1, 255, thickness=thickness)
    return img, black_img 

def get_templates():
    templates=[]
    for i in range(NUM_TEMPLATES):
        template = cv2.imread(os.path.join(PATH,"imgs/raw_templates/{}.PNG".format(i+1)),0)
        template = preprocessing(template)
        template = cv2.Canny(template,100,200)
        template,_ = drawContours(template, 3)
        templates.append(template)
    return templates

def template_matching(img):
    templates = get_templates()
    _, img = drawContours(img, 3)
    white_img = np.zeros((400,400), dtype=np.uint8)
    white_img[:] = 255
    plt.figure(figsize=(10,10))
    plt.subplot(121),plt.imshow(img, cmap='gray')
    plt.subplot(122),plt.imshow(white_img, cmap='gray', vmin=0, vmax=255)
    plt.show()
    locations = []
    for i in range(len(templates)):
        plt.figure(figsize=(10,10))
        plt.subplot(121),plt.imshow(templates[i], cmap='gray')
        plt.show()
        best_score = 1000000000000
        best_location = 0 
        sign_area = 0  
        for j in range(0,3):
            rotated_template = imutils.rotate_bound(templates[i].copy(), j)
            res = cv2.matchTemplate(img, rotated_template, cv2.TM_SQDIFF)
            _,max_val,min_loc,_ = cv2.minMaxLoc(res)
            if max_val < best_score:
                w,h = templates[i].shape[::-1]
                top_left = min_loc
                bottom_right = (top_left[0] + w, top_left[1] + h)
                sign_area = cv2.rectangle(white_img.copy(), top_left, bottom_right,0, -1)
                
                best_score = max_val
                best_location = [i+1, top_left[0] + w/2, top_left[1] + h/2]
        locations.append(best_location)
        white_img = sign_area
    print(locations)
    return white_img, locations

def get_sign(img):
    blur_img = cv2.GaussianBlur(img.copy(), (9,9), cv2.BORDER_CONSTANT)
    unsharpen_img = cv2.addWeighted(img.copy(), 2, blur_img, -1, 0, img.copy())
    edge_img = cv2.Canny(unsharpen_img.copy(), 100, 150)
    sign_area, sign_pos = template_matching(edge_img.copy())

    plt.figure(figsize=(10,10))
    plt.subplot(121),plt.imshow(edge_img, cmap='gray')
    plt.subplot(122),plt.imshow(sign_area, cmap='gray')
    plt.show()

    return sign_area, sign_pos

def get_area(img):
    blur_img = cv2.GaussianBlur(img.copy(), (9,9), cv2.BORDER_CONSTANT)
    unsharpen_img = cv2.addWeighted(img.copy(), 2, blur_img, -1, 0, img.copy())
    edge_img = cv2.Canny(unsharpen_img.copy(), 100, 150)
    _, draw_line_img = drawContours(edge_img.copy(), 3)
    _, fill_line_img = drawContours(draw_line_img.copy(), -1, 2000)
    sk_img = skeletonize(fill_line_img.copy(), method='lee')

    plt.figure(figsize=(10,10))
    plt.subplot(121),plt.imshow(fill_line_img, cmap='gray')
    plt.subplot(122),plt.imshow(sk_img, cmap='gray')
    plt.show()

    return fill_line_img, sk_img

def get_hsv(img):
    blur_img = cv2.medianBlur(img.copy(), 9)
    return cv2.cvtColor(blur_img, cv2.COLOR_BGR2HSV)

def get_map():
    # plt.figure(figsize=(10,10))
    # plt.subplot(121),plt.imshow(MAP_IMG, cmap='gray')
    # plt.subplot(122),plt.imshow(MAP_IMG_GRAY, cmap='gray')
    # plt.show()
    hsv_img = get_hsv(MAP_IMG.copy())
    sign_img, sign_pos = get_sign(MAP_IMG_GRAY.copy())
    area_img, sk_img = get_area(MAP_IMG_GRAY.copy())

    no_sign_img = cv2.bitwise_and(area_img, sign_img)

    plt.figure(figsize=(10,10))
    plt.subplot(231),plt.imshow(hsv_img, cmap='gray')
    plt.subplot(232),plt.imshow(sign_img, cmap='gray')
    plt.subplot(233),plt.imshow(area_img, cmap='gray')
    plt.subplot(234),plt.imshow(sk_img, cmap='gray')
    plt.subplot(235),plt.imshow(no_sign_img, cmap='gray')
    plt.subplot(236),plt.imshow(MAP_IMG_GRAY, cmap='gray')
    plt.savefig(PATH + '/imgs/ui/get_map.png')


    return hsv_img, area_img, no_sign_img, sk_img, sign_pos