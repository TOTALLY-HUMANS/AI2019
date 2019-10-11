#!/usr/bin/env python
import time
import math
import threading
import json
# import imutils
import cv2
import av
import numpy as np
from cv2 import aruco

from scipy.spatial import distance

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

#url = "udp://224.0.0.0:1234"
# print(cv2.getBuildInformation())


# Sentti on tämän verran pikseleitä
centimeter = 10

# ROBOT STATE DEFINITIONS
class RobotState(Enum):
    Idle = 0
    ChaseClosestRedBall = 1
    ChaseClosestGreenBall = 2
robot_1_id = 15
robot_2_id = 16
robot_1_state = RobotState.ChaseClosestGreenBall
robot_2_state = RobotState.ChaseClosestRedBall
own_goal_pose = (0, 0)
opponent_goal_pose = (1080, 1080)
ROBOT_SIZE = 100

# MAIN LOOPERO
def main():
    print("Loading configuration.")
    with open('config.json') as json_data:
        config = json.load(json_data)
    #SI1 = socketInterface()
    #SI2 = socketInterface()

    print("Connecting to camera.")
    #cap = AvVideoCapture(config["url"])
    #cap = VideoCapture(url)
    cap = cv2.VideoCapture('videos/Balls1.ts')
    print("Initializing ball detector.")
    ball_detector = BallDetector(config["ball_detector"], debug=False)
    print("Initalizing ball tracker.")
    ball_tracker = EuclidianTracker(10)
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
                tracked_balls = ball_tracker.update(balls)

                corners, ids = aruco_detector.get_arucos(img)
                positions = aruco_detector.get_positions(corners, ids)
                
                visualize_detected(img, tracked_balls, corners, ids, positions)
                
            

                # Run robot AI
                #evaluateRobotState(15, tracked_balls, positions)
                #evaluateRobotState(16, tracked_balls, positions)

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
        color = (127, 0, 255)
        if val.color == 1:
            color = (0, 179, 255)
        #Draw the centroid and a circle around the ball
        cv2.circle(img, val.center, 2, color, 2)
        cv2.circle(img, val.center, val.radius, color, 2)
        #Draw a speed vector from the ball
        speed = tuple(np.array(val.center)+np.array(val.speed))
        cv2.line(img, val.center, (int(speed[0]),int(speed[1])),(0,0,255),2)
        coordinates = coordinatesForRobotBehindBall(val)
        cv2.circle(img, (int(coordinates[0]), int(coordinates[1])) , 2, (255, 0, 0), 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(key), val.center, font, 1, (0, 255, 0), 2, cv2.LINE_AA)
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
    if robot == 16:
        currentRobotState = robot_2_state
    robotStates = {
        0: Idle,
        1: ChaseClosestRedBall,
        2: ChaseClosestGreenBall,
    }
    robot_pose = (0.0, 0.0, 0.0)
    pose_found = False
    for pos in robot_positions:
        if pos[3] == robot:
            robot_pose = pos
            pose_found = True
    if not pose_found:
        print("No robot found, idling")
        updateState(robot, RobotState.Idle)
    robotStates[currentRobotState](robot, ball_positions, robot_pose)

def updateState(robot, newState):
    if robot == robot_1_id:
        robot_1_state = newState
    if robot == robot_2_id:
        robot_2_state = newState

# STATES

def Idle(robot, tracked, robot_pose):
    print("Idling")
    updateState(robot, RobotState.ChaseClosestRedBall)

def ChaseClosestRedBall(robot, tracked, robot_pose):
    print("Chasing red balls...")
    if len(tracked) == 0:
        print("No balls found, idling")
        updateState(robot, RobotState.Idle)
        return
    target = getClosestBall(tracked, robot_pose, -1)
    moveTowardsTarget(robot, target, robot_pose)
    if isNearTarget(robot_pose, target):
        updateState(robot, RobotState.PrepareToHitRedBall)

def ChaseClosestGreenBall(robot, tracked, robot_pose):
    print("Chasing green balls...")
    if len(tracked) == 0:
        print("No balls found, idling")
        updateState(robot, RobotState.Idle)
        return
    target = getClosestBall(tracked, robot_pose, 1)
    moveTowardsTarget(robot, target, robot_pose)
    if isNearTarget(robot_pose, target):
        updateState(robot, RobotState.PrepareToHitGreenBall)

# HELPER METHODS

"""
Return coordinates behind the ball based on the color of the ball and the position of the goal

Input: 
    a ball (x, y, ballType)
"""
def coordinatesForRobotBehindBall(ball):
    x,y = ball.center
    color = ball.color

    goal_pose = None
    if (color == 1):
        goal_pose = opponent_goal_pose
    elif (color == -1):
        goal_pose = own_goal_pose

    vector_to_goal = np.array([goal_pose[0] - x, goal_pose[1] - y])
    vector_magnitude = np.linalg.norm(vector_to_goal)
    position_behind_ball = np.array([x, y]) - (vector_to_goal/vector_magnitude) * ROBOT_SIZE
    
    return position_behind_ball


def isNearTarget(robot_pose, target):
    dist = distance.euclidean((target[0], target[1]), (robot_pose[0], robot_pose[1]))
    if dist < centimeter * 10:
        return True
    return False

def getClosestBall(tracked, robot_pose, ballType):
    chosenBall = tracked[0]
    shortestDistance = 100000
    for ball in tracked.values():
        if (ball[3] == ballType):
            dist = distance.euclidean((ball[0], ball[1]), (robot_pose[0], robot_pose[1]))
            if (dist < shortestDistance):
                shortestDistance = dist
                chosenBall = ball
    return chosenBall

def moveTowardsTarget(robot, ball_pose, robot_pose):
    ball_x = ball_pose[0]
    ball_y = ball_pose[1]
    robot_x = robot_pose[0]
    robot_y = robot_pose[1]
    robot_yaw = robot_pose[2]

    # ohjauskomento
    r_com = 0.0
    l_com = 0.0
    deadzone = 90
    brakezone = 50

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
    if robot == robot_1_id:
        SI1.send_command(r_com, l_com)
    if robot == robot_2_id:
        SI2.send_command(r_com, l_com)

if __name__ == "__main__":
    main()
