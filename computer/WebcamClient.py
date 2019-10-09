#!/usr/bin/env python
import time
import math
import threading
# import imutils
import cv2
import av
import numpy as np
from cv2 import aruco

from av_video_capture import AvVideoCapture
from video_capture import VideoCapture

from ball_detector import BallDetector
from aruco_detector import ArucoDetector
from euclidian_tracker import EuclidianTracker

from my_robot import drive_commands
from socket_interface import socketInterface


# Greenish yellow hsv
yellow_low = np.array([25, 50, 160])
yellow_high = np.array([35, 255, 255])
# Pink hsv
pink_low = np.array([150, 50, 160])
pink_high = np.array([175, 255, 255])

radius_range = [13, 16]

url = "udp://224.0.0.0:1234"
# print(cv2.getBuildInformation())


def main():
    #SI = socketInterface()

    print("Connecting to camera")
    #cap = AvVideoCapture(url)
    #cap = VideoCapture(url)
    cap = cv2.VideoCapture('videos/Balls2.ts')
    print("Initializing ball detector.")
    ball_detector = BallDetector(yellow_low, yellow_high, pink_low,
                                 pink_high, ballSizeRange=radius_range, debug=False)
    print("Initalizing tracker.")
    tracker = EuclidianTracker(10)
    print("Initializing aruco detector")
    aruco_detector = ArucoDetector()
    try:
        print("trying")
        while 1:
            time1 = time.time()
            ret, img = cap.read()

            if ret:

                
                img = downscale_image(img, 90)

                balls = ball_detector.detect_balls(img)
                tracked = tracker.update(balls)


                corners, ids = aruco_detector.get_arucos(img)
                positions = aruco_detector.get_positions(corners, ids)
                
                visualize_detected(img, tracked, corners, ids, positions)
                
            

                
                '''



                #for i in ids:
                    #print("id: ", str(i))
                robot_pose = (0.0, 0.0, 0.0)
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

                    # ohjauskomento
                    r_com = 0.0
                    l_com = 0.0
                    deadzone = 90
                    brakezone = 50
                    if ball_found:
                        r_com, l_com = drive_commands(
                            ball_x, ball_y, robot_x, robot_y, robot_yaw)
                        r_com = 150*r_com #255*r_com
                        l_com = 150*l_com #255*l_com

                        if abs(r_com) < brakezone:
                            r_com = 0
                        elif abs(r_com) < deadzone:
                            if r_com < 0:
                                r_com = r_com - deadzone
                            if r_com > 0:
                                r_com = r_com + deadzone
                        if abs(l_com) < brakezone:
                            l_com = 0
                        elif abs(l_com) < deadzone:
                            if l_com < 0:
                                l_com = l_com - deadzone
                            if r_com > 0:
                                l_com = l_com + deadzone

                    # ohjauskomento sokettiin
                    SI.send_command(r_com, l_com)
                    '''
            time2 = time.time()
            frame_time = (time2-time1)*1000
            if frame_time != 0:
              fps = 1000/frame_time
            print("Frame_time:",frame_time, 'ms','fps:', fps)
    except:
        raise
    finally:
        print("closed.")


def downscale_image(img, scale_percent):
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)

    return cv2.resize(img, dim, interpolation=cv2.INTER_AREA)


def visualize_detected(img, balls, aruco_corners, aruco_ids, positions):
    for key,val in balls.items():
        x, y, r, v = val
        color = (127, 0, 255)
        if v == 1:
            color = (0, 179, 255)
        cv2.circle(img, (x, y), 2, color, 2)
        cv2.circle(img, (x, y), r, color, 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(key), (x,y), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
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
