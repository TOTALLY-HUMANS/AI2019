#!/usr/bin/env python
import socket
import time
import math

import cv2
import numpy as np

from ball_detector import BallDetector

ip_video = 'localhost'
port_video = 5005

ip_robot = 'localhost'
port_robot = 5006

BUFFER_SIZE = 1000000

'''
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
    frame = cv2.GaussianBlur(frame,(5,5),0);

    # Convert the colorspace to HSV (Hue, Saturation, Value)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    return cv2.inRange(hsv, low_b, high_b)

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
    client_video = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_video.connect((ip_video, port_video))

    print("Initializing feature detector.")

    #fast = cv2.FastFeatureDetector_create()

    print( "Initializing server")

    server_robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_robot.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_robot.bind((ip_robot, port_robot))
    server_robot.listen(1)

    print( "Waiting for robot")

    # Wait for robot to connect
    roboconn, addr = server_robot.accept()

    #Greenish yellow hsv (25,100,150) ~ (35, 255, 255)
    yellow_low = (25,40,50)
    yellow_high = (40,255,255)
    #Pink (150, 100, 150)  ~ (160, 255, 255)
    pink_low = (150,100,50)
    pink_high = (160,255,255)

    radius_range = (5, 8)

    detector = BallDetector(yellow_low,yellow_high, pink_low, pink_high,ballSizeRange=radius_range, debug=False)


    try:
        print("trying")
        while 1:
            print("Listening for image...")
            img = client_video.recv(BUFFER_SIZE)
            time.sleep(1)
            print("Checking if image has been received.")
            if not img:
                print("no image")
                break
            print("Checking if length is sufficient...")
            while not ":".encode() in img:
                img = client_video.recv(BUFFER_SIZE)
            print("Image decoding magic...")
            img = img.decode(encoding="latin-1")
            img_full_length = int(img.partition(":")[0])
            img = img.partition(":")[2].encode("latin-1")
            print(img_full_length)
            while len(img) < img_full_length:
                bytes = client_video.recv(BUFFER_SIZE)
                img += bytes
            img = img[:img_full_length] # Truncate in case there's too much data.
            
            print("received mockup img.")
            print(len(img))
            img = np.frombuffer(img, dtype=np.uint8).reshape(750,750,3)
            print(len(img))
            '''        
            yellow_img = cv2.bitwise_and(img,img, mask= maskFrame(img, yellow_low, yellow_high))
            #cv2.imwrite('yellow_masked.png', yellow_img)
            pink_img = cv2.bitwise_and(img,img, mask= maskFrame(img, pink_low,pink_high))
            #cv2.imwrite('pink_masked.png', pink_img)

            yellow_balls = detect_balls(fast, yellow_img, 1)
            pink_balls = detect_balls(fast, pink_img, -1)

            hough = hough_detect_balls(pink_img, -1)
            print(hough)
            

            balls = [*yellow_balls,*pink_balls]
            '''
            balls = detector.detect_balls(img)
    
            
           

            balls = str(balls).encode("latin-1")

            print("Sending...")

            roboconn.send(balls)
    except:
        raise
    finally:
        client_video.close()
        roboconn.close()
        print( "closed.")

if __name__ == "__main__":
    main()
