import cv2
import math
import time
import threading
import numpy as np
import yaml_handle as yaml_handle


range_rgb = {
    'red':   (0, 0, 255),
    'blue':  (255, 0, 0),
    'green': (0, 255, 0),
    'black': (0, 0, 0),
    'white': (255, 255, 255)}

__target_color = ('red', 'green', 'blue')
lab_data = yaml_handle.get_yaml_data(yaml_handle.lab_file_path)

def getAreaMaxContour(contours):
    """Find contour of the max area.

    Args:
        contours (_list_): collection of contours for comparison

    Returns:
       area_max_contour, contour_area_max
    """
    contour_area_temp = 0
    contour_area_max = 0
    area_max_contour = None

    for c in contours:  
        contour_area_temp = math.fabs(cv2.contourArea(c))  # Calculate contour area
        if contour_area_temp > contour_area_max:
            contour_area_max = contour_area_temp
            if contour_area_temp > 300:  # Filter out noise
                area_max_contour = c

    return area_max_contour, contour_area_max  # return the max area and the corresponding contour

detect_color = None
color_list = []
start_pick_up = False
size = (640, 480)
def run(img):
    global rect
    global detect_color
    global start_pick_up
    global color_list
        
    img_copy = img.copy()
    frame_resize = cv2.resize(img_copy, size, interpolation=cv2.INTER_NEAREST)
    frame_gb = cv2.GaussianBlur(frame_resize, (3, 3), 3)
    frame_lab = cv2.cvtColor(frame_gb, cv2.COLOR_BGR2LAB)  # convert to LAB space
    color_area_max = None
    max_area = 0
    areaMaxContour_max = 0
    if not start_pick_up:
        for i in lab_data:
            if i in __target_color:
                frame_mask = cv2.inRange(frame_lab,
                                             (lab_data[i]['min'][0],
                                              lab_data[i]['min'][1],
                                              lab_data[i]['min'][2]),
                                             (lab_data[i]['max'][0],
                                              lab_data[i]['max'][1],
                                              lab_data[i]['max'][2])) 
                opened = cv2.morphologyEx(frame_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)) 
                closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))  
                contours = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  
                areaMaxContour, area_max = getAreaMaxContour(contours)  
                
                if areaMaxContour is not None:
                    if area_max > max_area: 
                        max_area = area_max
                        color_area_max = i
                        areaMaxContour_max = areaMaxContour
        
        if max_area > 500:  
            rect = cv2.minAreaRect(areaMaxContour_max)
            box = np.int0(cv2.boxPoints(rect))
            y = int((box[1][0]-box[0][0])/2+box[0][0])
            x = int((box[2][1]-box[0][1])/2+box[0][1])
            print('X:',x,'Y:',y) 
            cv2.drawContours(img, [box], -1, range_rgb[color_area_max], 2)
           
    return img

if __name__ == '__main__':
    
    cap = cv2.VideoCapture(-1) 
    __target_color = ('blue',)
    while True:
        ret, img = cap.read()
        if ret:
            frame = img.copy()
            Frame = run(frame)           
            cv2.imshow('Frame', Frame)
            key = cv2.waitKey(1)
            if key == 27:
                break
        else:
            time.sleep(0.01)
    cv2.destroyAllWindows()

        