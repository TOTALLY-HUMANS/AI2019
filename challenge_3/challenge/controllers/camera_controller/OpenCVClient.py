#!/usr/bin/env python
import socket
import time
import math

import cv2
import numpy as np

ip_video = 'localhost'
port_video = 5005

ip_robot = 'localhost'
port_robot = 5006

BUFFER_SIZE = 1000000

def detect_balls(fast, img):
    """Detect balls in image, return coordinates as list."""

    kps = fast.detect(img,None)
    kps = merge_keypoints(kps, 20)

    return [kp.pt for kp in kps]

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

def maskFrame(frame):
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

    # We want to look for bright, multicolored balls. That means we want to extract parts of the image with:
    # Any hue
    # High saturation
    # High value
    #Greenish yellow (25,100,20) ~ (35, 255, 255)
    yellow_mask = cv2.inRange(hsv, (25,100,20), (35,255,255))
    return yellow_mask
    
'''return cv2.inRange(hsv,
                       np.array([0,180,180]),
                       np.array([255,255,255]))'''

def main():
    print("Connecting to camera")
    client_video = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_video.connect((ip_video, port_video))

    print("Initializing feature detector.")

    fast = cv2.FastFeatureDetector_create()

    print( "Initializing server")

    server_robot = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_robot.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_robot.bind((ip_robot, port_robot))
    server_robot.listen(1)

    print( "Waiting for robot")

    # Wait for robot to connect
    roboconn, addr = server_robot.accept()

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

            img = cv2.bitwise_and(img,img, mask= maskFrame(img))
            cv2.imwrite('masked.png', img)
            balls = detect_balls(fast, img)

            print(balls)

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
