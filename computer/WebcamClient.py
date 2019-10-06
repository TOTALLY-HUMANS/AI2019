#!/usr/bin/env python
import time
import math
#import imutils
import cv2
import numpy as np
from cv2 import aruco
from ball_detector import BallDetector
from aruco_detector import ArucoDetector

from my_robot import drive_commands
from socket_interface import socketInterface

# Greenish yellow hsv (25,100,150) ~ (35, 255, 255)
yellow_low = np.array([25, 50, 160])
yellow_high = np.array([35, 255, 255])
# Pink (150, 100, 150)  ~ (160, 255, 255)
pink_low = np.array([150, 50, 160])
pink_high = np.array([175, 255, 255])

radius_range = [13, 30]

url = "udp://224.0.0.0:1234"
    
def main():
    SI = socketInterface()
    
    print("Connecting to camera")
    camera = cv2.VideoCapture(url)
    #camera.set(cv2.CV_CAP_PROP_BUFFERSIZE,1)
    s = camera.set(cv2.CAP_PROP_BUFFERSIZE, 3)

    width = camera.get(cv2.CAP_PROP_FRAME_WIDTH)   # float
    height = camera.get(cv2.CAP_PROP_FRAME_HEIGHT)
    print("Connected to camera, resolution is %ix%i" % (width, height))


    print("Initializing ball detector.")
    ball_detector = BallDetector(yellow_low, yellow_high, pink_low,
                                 pink_high, ballSizeRange=radius_range, debug=False)
    print("Initializing aruco detector")
    aruco_detector = ArucoDetector()
    skip = 0

    try:
        print("trying")
        while 1:
            #print("Listening for image...")  
            skip +=1
            if skip%5 == 0:
                camera.release()
                camera.open(url)
                ret, img = camera.read()
                if img is None:
                    print("No frame")

                    camera.release()
                    camera.open(url)
        
                if ret:
                    
                    balls = ball_detector.detect_balls(img)

                    corners, ids = aruco_detector.get_arucos(img)
                    positions = aruco_detector.get_positions(corners,ids)

                    visualize_detected(img, balls, corners, ids, positions)
                    
                    for i in ids:
                      print("id: " , str(i))
                    
                    #cv2.imshow('img',img)
                    robot_pose = (0.0,0.0,0.0)
                    for pos in positions:

                      if pos[3] == 16:
                        robot_pose = pos
                    ball_found = False
                    if len(balls) > 0:
                      ball_pose = balls[0]
                      ball_x = ball_pose[0]
                      ball_y = ball_pose[1]
                      ball_found = True

                    robot_x = robot_pose[0]
                    robot_y = robot_pose[1]
                    robot_yaw = robot_pose[2]
                    
                                        #ohjauskomento
                    
                    if ball_found:
                      r_com, l_com = drive_commands(ball_x, ball_y, robot_x, robot_y, robot_yaw)
                    
                    #ohjauskomento sokettiin
                    SI.send_command(128*r_com, 128*l_com)
            
    except:
        raise
    finally:
        print("closed.")


def visualize_detected(img, balls, aruco_corners, aruco_ids, positions):
    for ball in balls:
        x, y, r, v = ball
        color = (127, 0, 255)
        if v == 1:
            color = (0, 179, 255)
        cv2.circle(img, (x,y), 2, color, 2)
        cv2.circle(img, (x, y), r, color, 2)
    aruco.drawDetectedMarkers(img, aruco_corners, aruco_ids)
    for p in positions:
        x, z, theta, id = p
        x = int(x)
        z = int(z)
        center = (x, z)
        length = 20
        forward = (int(x+length*np.cos(theta)), int(z+length*np.sin(theta)))
        color = (0, 0, 255)
        cv2.circle(img, center, 2, color, 2)
        cv2.line(img, center, forward, color, 2)
    cv2.imshow('Detected', img)
    key = cv2.waitKey(1)


if __name__ == "__main__":
    main()
