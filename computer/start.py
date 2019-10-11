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
from ultrasonic_capture import UltrasonicCapture

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
    FindTarget = 1
    ChaseTarget = 2
    PrepareToHitTarget = 3
    HitTarget = 4
robot_1_id = 15
robot_2_id = 16
robot_1_state = RobotState.Idle
robot_2_state = RobotState.Idle
robot_1_target = None
robot_2_target = None
UltrasonicSensor = None
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

                balls,mask = ball_detector.detect_balls(img)
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

# Evaluoi robotin tilaa, pyörittää tilakonetta
def evaluateRobotState(robot, ball_positions, robot_positions):
    if robot == 15:
        currentRobotState = robot_1_state
    if robot == 16:
        currentRobotState = robot_2_state
    robotStates = {
        0: Idle,
        1: FindTarget,
        2: ChaseTarget,
        3: PrepareToHitTarget,
        4: HitTarget,
    }
    robot_pose = (0.0, 0.0, 0.0)
    pose_found = False
    for pos in robot_positions:
        if pos[3] == robot:
            robot_pose = pos
            pose_found = True
    if not pose_found:
        print(robot + ": No robot found, idling")
        updateState(robot, RobotState.Idle)
    robotStates[currentRobotState](robot, ball_positions, robot_pose)

# Vaihdetaan tilakoneen tilaa
def updateState(robot, newState):
    if robot == robot_1_id:
        robot_1_state = newState
    if robot == robot_2_id:
        robot_2_state = newState

# STATES

# Robotti idlaa oletustilassaan, ja koettaa löytää kohteen
def Idle(robot, tracked, robot_pose):
    print(robot + ": Idling")
    updateState(robot, RobotState.FindTarget)

# Robotti etsii kohteen
def FindTarget(robot, tracked, robot_pose):
    print(robot + ": Finding target...")
    if len(tracked) == 0: # Palloja ei löydy, idlataan
        print(robot + ": No balls found, idling")
        updateState(robot, RobotState.Idle)
        return
    if robot == robot_1_id: # Ykkösrobo jahtaa punaista
        robot_1_target = target = getClosestBall(tracked, robot_pose, -1)
    if robot == robot_2_id: # Kakkosrobo jahtaa keltaista
        robot_2_target = target = getClosestBall(tracked, robot_pose, 1)
    if target is not None:
        updateState(robot, RobotState.ChaseTarget) # Kohde löytyy, lähdetään perään
    else:
        updateState(robot, RobotState.Idle) # Kohdetta ei löydy, idlataan

# Ajetaan pallon tyypistä riippuen sen eteen tai taakse
def ChaseTarget(robot, tracked, robot_pose):
    print(robot + ": Chasing target...")
    target = getTarget(robot)
    # Liikutaan pallon taakse
    moveTowardsTarget(robot, coordinatesForRobotBehindBall(target), robot_pose)
    # Jos ollaan riittävän lähellä palloa, tähdätään siihen
    if isNearTarget(robot_pose, coordinatesForRobotBehindBall(target)):
        updateState(robot, RobotState.PrepareToHitTarget)

# Tähdätään palloon
def PrepareToHitTarget(robot, tracked, robot_pose):
    print(robot + ": Preparing to hit target...")
    # Käännytään kohti palloa
    target = getTarget(robot)
    rotateTowardsTarget(robot, target)
    # Jos ollaan riittävän lähellä, jyrätään päin
    if UltrasonicCapture.read() < 20 * centimeter and isNearTarget(robot_pose, target):
        updateState(robot, RobotState.HitTarget)
    
# Jyrätään palloon
def HitTarget(robot, tracked, robot_pose):
    print(robot + ": Hitting target...")
    # Ajetaan päin
    ramForward(robot)
    # Pallo karkasi, palataan idlaamaan (ja etsimään uutta kohdetta)
    if UltrasonicCapture.read() > 30 * centimeter:
        # Nollataan kohde
        if robot == robot_1_id:
            robot_1_target = None
        if robot == robot_2_id:
            robot_2_target = None
        updateState(robot, RobotState.Idle)

# HELPER METHODS

# Luetaan kohde
def getTarget(robot):
    if robot == robot_1_id:
        return robot_1_target
    if robot == robot_2_id:
        return robot_2_target

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
    chosenBall = None
    shortestDistance = 100000
    for ball in tracked.values():
        if (ball.color == ballType):
            dist = distance.euclidean((ball[0], ball[1]), (robot_pose[0], robot_pose[1]))
            if (dist < shortestDistance):
                shortestDistance = dist
                chosenBall = ball
    return chosenBall

# ROBOTIN LIIKKUMINEN

def ramForward(robot):
    print("Nyt mennään")
    # AJETAAN LUJAA PÄIN

def rotateTowardsTarget(robot, target):
    print("Käännytään targettiin")
    # Käännytään kohti targetia

def moveTowardsTarget(robot, target_pose, robot_pose):
    target_x = target_pose[0]
    target_y = target_pose[1]
    robot_x = robot_pose[0]
    robot_y = robot_pose[1]
    robot_yaw = robot_pose[2]

    # ohjauskomento
    r_com = 0.0
    l_com = 0.0
    deadzone = 90
    brakezone = 50

    r_com, l_com = drive_commands(
        target_x, target_y, robot_x, robot_y, robot_yaw)
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
