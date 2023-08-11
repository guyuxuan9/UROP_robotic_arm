import math
import cv2

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

def rescale(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min