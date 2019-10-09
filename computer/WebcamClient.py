#!/usr/bin/env python
import time
import math
import threading
# import imutils
import cv2
import av
import numpy as np
from cv2 import aruco

from enum import Enum

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

# ROBOT STATE DEFINITIONS
class RobotState(Enum):
    Idle = 0
    ChaseClosestRedBall = 1
    ChaseClosestGreenBall = 2
robot_1_state = RobotState.ChaseClosestGreenBall
robot_2_state = RobotState.ChaseClosestRedBall

# MAIN LOOPERO
def main():
    SI15 = socketInterface()
    SI16 = socketInterface()

    print("Connecting to camera")
    #cap = AvVideoCapture(url)
    #cap = VideoCapture(url)
    cap = cv2.VideoCapture('videos/Balls1.ts')
    print("Initializing ball detector.")
    ball_detector = BallDetector(yellow_low, yellow_high, pink_low,
                                 pink_high, ballSizeRange=radius_range, debug=False)
    print("Initalizing tracker.")
    tracker = EuclidianTracker(2000)
    print("Initializing aruco detector")
    aruco_detector = ArucoDetector()
    try:
        print("trying")
        while 1:
            time1 = time.time()
            ret, img = cap.read()
           
            #cv2.imshow('test', img)
            #key = cv2.waitKey(1)

            if ret:

                
                img = downscale_image(img, 90)

                # img = downscale_image(img,80)
                balls = ball_detector.detect_balls(img)
                balls = [b for b in balls if b[3]==1]
                print(len(balls))
                tracked = tracker.update(balls)
                print(tracked)

                corners, ids = aruco_detector.get_arucos(img)
                positions = aruco_detector.get_positions(corners, ids)
                if len(tracked) >= 3:
                    visualize_detected(img, tracked, corners, ids, positions)
                    cv2.imwrite('wtf.jpg',img)
                    break

                # Run robot AI
                evaluateRobotState(15, tracked, positions)
                evaluateRobotState(16, tracked, positions)

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

### HERE BEGINS ROBOT AI ###

def evaluateRobotState(robot, ball_positions, robot_positions):
    if robot == 15:
        currentRobotState = robot_1_state
        currentRobotSocket = SI15
    if robot == 16:
        currentRobotState = robot_2_state
        currentRobotSocket = SI16
    robotStates = {
        0: Idle,
        1: ChaseClosestRedBall,
        2: ChaseClosestGreenBall,
    }
    robotStates[currentRobotState](robot, tracked, positions, socket)

def updateState(robot, newState):
    if robot == 15:
        robot_1_state = newState
    if robot == 16:
        robot_2_state = newState

def Idle(robot, tracked, positions, socket):
    print("Idling")
    updateState(robot, RobotState.ChaseClosestGreenBall)

def ChaseClosestRedBall(robot, tracked, positions, socket):
    print("Chasing red balls...")
    robot_pose = (0.0, 0.0, 0.0)
    pose_found = False
    for pos in positions:
        if pos[3] == robot:
            robot_pose = pos
            pose_found = True
    if not pose_found:
        print("No ball found")
        return

    ball_found = False
    if len(tracked) > 0:
        ball_pose = tracked[0]
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
    socket.send_command(r_com, l_com)

def ChaseClosestGreenBall(robot, tracked, positions, socket):
    print("Chasing green balls...")


if __name__ == "__main__":
    main()
