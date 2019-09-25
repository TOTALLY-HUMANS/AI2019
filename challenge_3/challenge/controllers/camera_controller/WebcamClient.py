#!/usr/bin/env python
import time
import math
import imutils
import cv2
import numpy as np
from ball_detector import BallDetector

'''
def hough_detect_balls(img,value):
    gray = img
    #gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imshow('gray', gray)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, img.shape[0]/64, param1=200, param2=10, minRadius=15, maxRadius=20)
    if circles is None:
        print("No circles found")
        return []
    print(circles)
    circles = np.round(circles[0, :]).astype("int")
    return [x for x in circles]

def detect_balls(fast, img, value):
    """Detect balls in image, return coordinates as list."""

    kps = fast.detect(img,None)
    kps = merge_keypoints(kps, 20)

    return [kp.pt + (value,) for kp in kps]

def distance_between_points(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def has_close_points(point, others, radius):
    for other in others:
        if (distance_between_points(point.pt, other.pt) < radius):
            return True
    return False

def merge_keypoints(keypoints, radius):
    result = []
    for point in keypoints:
        if (not has_close_points(point, result, radius)):
            result += [point]

    return result

def contour_detect_balls(img, value):

    binary = cv2.Canny(img.copy(),100,200)
    cv2.imshow('masked', img)
    cv2.imshow('binary', binary)
    contours = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = imutils.grab_contours(contours)
    if len(contours) > 0:
        return [cv2.minEnclosingCircle(contour) for contour in contours]


def maskFrame(frame, low_b, high_b):
    """
    Perform basic preprocessing to create a mask that can be overlayed over the
    image. This will dramatically reduce the search space for more complicated
    operations further down the pipeline, speeding up computation.

    :param frame: The imput image
    :return: A binary mask corresponding to parts of the image that have
    similar hue, saturation, and value levels as the objects to be detected
    """
    # Blur the image to reduce high frequency noise
    #frame = cv2.GaussianBlur(frame,(7,7),0);
    frame = cv2.medianBlur(frame,5)

    # Convert the colorspace to HSV (Hue, Saturation, Value)
    mask = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Mask frame  to contain wanted colors.
    mask = cv2.inRange(mask, low_b, high_b)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask,None, iterations=2)

    return mask

    # We want to look for bright, multicolored balls. That means we want to extract parts of the image with:
    # Any hue
    # High saturation
    # High value
    return cv2.inRange(hsv,
                       np.array([0,180,180]),
                       np.array([255,255,255]))
'''
def main():
    print("Connecting to camera")
    camera = cv2.VideoCapture(0)

    print("Initializing feature detector.")

    #fast = cv2.FastFeatureDetector_create()

    #Greenish yellow hsv (25,100,150) ~ (35, 255, 255)
    yellow_low = (25,50,100)
    yellow_high = (40,255,255)
    #Pink (150, 100, 150)  ~ (160, 255, 255)
    pink_low = (150,50,80)
    pink_high = (195,255,255)

    radius_range = (13, 30)

    detector = BallDetector(yellow_low,yellow_high, pink_low, pink_high,ballSizeRange=radius_range, debug=True)


    try:
        print("trying")
        while 1:
            print("Listening for image...")
            ret, img = camera.read()      
            balls = detector.detect_balls(img)
            print(balls) 
            time.sleep(1)    
    except:
        raise
    finally:
        print( "closed.")

if __name__ == "__main__":
    main()
