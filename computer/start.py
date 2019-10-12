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
from math import acos
from math import sqrt
from math import pi
#import vg

from scipy.spatial import distance

from enum import IntEnum

from av_video_capture import AvVideoCapture
from video_capture import VideoCapture

from ball_detector import BallDetector
from aruco_detector import ArucoDetector
from aruco_tracker import ArucoTracker
from euclidian_tracker import EuclidianTracker

from my_robot import drive_commands
from socket_interface import socketInterface

from astar import astar

# Greenish yellow hsv
yellow_low = np.array([25, 50, 160])
yellow_high = np.array([35, 255, 255])
# Pink hsv
pink_low = np.array([150, 50, 160])
pink_high = np.array([175, 255, 255])

radius_range = [13, 16]

#url = "udp://224.0.0.0:1234"
# print(cv2.getBuildInformation())


#sentti pikseleiss
centimeter = (1080*(90/100))/402

# ROBOT STATE DEFINITIONS
class RobotState(IntEnum):
    Idle = 0
    FindTarget = 1
    ChaseTarget = 2
    PushBallToGoal = 3
    TurnTowardsTarget = 4

robot_1_id = 16
robot_2_id = 17
robot_1_state = RobotState.Idle
robot_2_state = RobotState.Idle
robot_1_target = None
robot_2_target = None
robot_1_target_id = None
robot_2_target_id = None
robot_1_path = None
robot_2_path = None
robot_1_path_current_node = None
robot_2_path_current_node = None
SI1 = None
SI2 = None
UltrasonicSensor = None
own_goal_pose = (0, 0)
opponent_goal_pose = (972, 972)
ROBOT_SIZE = 100
mask = None
wall_correction = 20

# MAIN LOOPERO
def main():
    global SI1
    global SI2
    global mask
    print("Loading configuration.")
    with open('config.json') as json_data:
        config = json.load(json_data)
    SI1 = socketInterface()
    #SI2 = socketInterface()

  
    count = 0
    print("Connecting to camera.")
    cap = AvVideoCapture(config["url"])
    #cap = VideoCapture(url)
    #cap = cv2.VideoCapture('videos/Balls1.ts')
    print("Initializing ball detector.")
    ball_detector = BallDetector(config["ball_detector"], debug=False)
    print("Initalizing ball tracker.")
    ball_tracker = EuclidianTracker(10)
    print("Initializing aruco detector")
    aruco_detector = ArucoDetector()
    print("Initializing aruco tracker")
    aruco_tracker = ArucoTracker()
    try:
        print("trying")
        while 1:
            time1 = time.time()
            ret, img = cap.read()
            count += 1
            if ret:
                img = downscale_image(img, 90)

                balls, mask = ball_detector.detect_balls(img)
                tracked_balls = ball_tracker.update(balls)

                corners, ids = aruco_detector.get_arucos(img)
                positions = aruco_detector.get_positions(corners, ids)
                aruco_positions = aruco_tracker.update(positions)

                #mask = evaluateStageMask(mask, positions, 200)

                # find path
                start_time = time.time()
                elapsed_time = time.time() - start_time
                #print("astar time:" + str(elapsed_time))
                #path_x = [i[0] for i in path] 
                #path_y = [i[1] for i in path]
                #mask[path_x, path_y] = 125
                
                visualize_detected(img, tracked_balls, corners, ids, aruco_positions)
                #print(aruco_positions)
                # Run robot AI
                evaluateRobotState(robot_1_id, tracked_balls, aruco_positions)
                #evaluateRobotState(robot_2_id, tracked_balls, positions)
                
                '''robot_pose = (0.0, 0.0, 0.0)
                for key,pos in aruco_positions.items():

                    if pos[3] == 16:
                        robot_pose = pos
                    ball_found = False
                    if len(balls) > 0:
                        ball_pose = balls[0].center
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
                        r_com = 170*r_com #255*r_com
                        l_com = 170*l_com #255*l_com
                        print(l_com,r_com)
                        
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
                    SI1.send_command(r_com, l_com)'''
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
        coordinates = coordinatesForRobotBehindBall(val,wall_correction)
        cv2.circle(img, (int(coordinates[0]), int(coordinates[1])) , 2, (255, 0, 0), 2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, str(key), val.center, font, 1, (0, 255, 0), 2, cv2.LINE_AA)
    aruco.drawDetectedMarkers(img, aruco_corners, aruco_ids)
    for key,p in positions.items():
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


def evaluateStageMask(mask, positions, resize_side):
    # remove balls
    mask[mask == 255] = 0

    # create masks for robots
    for p in positions:
        #print(p)
        x = p[0]
        y = p[1]
        theta = p[2]
        c,s = np.cos(theta), np.sin(theta)
        t = np.array([x,y])
        rot_mat = np.array((c, -s),(s, c))
        for i in range(-30,30):
            for j in range(-30,30):
                trans_i = i*c +s*j
                trans_j = -i*s +c*j
                mask[int(trans_i+y),int(trans_j+x)] = 255

    # dilate balls and robot, resize mask
    center_x = 500 # center of stage
    center_y = 450
    theta = 7.1
    c,s = np.cos(theta), np.sin(theta)
    t = np.array([center_x,center_y])
    rot_mat = np.array((c, -s),(s, c))
    for i2 in range(-150,150):
        for j2 in range(-150,150):
            trans_i = i2*c +s*j2
            trans_j = -i2*s +c*j2
            mask[int(trans_i+center_y),int(trans_j+center_x)] = 150

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))  
    mask = cv2.dilate(mask, kernel, iterations=10)
    mask = cv2.resize(mask, (int(972*0.2), int(972*0.2)))

    return mask

### HERE BEGINS ROBOT AI ###

# STATES

# Robotti idlaa oletustilassaan, ja koettaa loytaa kohteen
def Idle(robot, tracked, robot_pose):
    print(str(robot) + ": Idling")
    updateState(robot, RobotState.FindTarget)

# Robotti etsii kohteen
def FindTarget(robot, tracked, robot_pose):
    global robot_1_target
    global robot_2_target
    global robot_1_target_id
    global robot_2_target_id
    global robot_1_path
    global robot_1_path_current_node
    global robot_2_path
    global robot_2_path_current_node
    global mask

    print(str(robot) + ": Finding target...")
    if len(tracked) == 0: # Palloja ei loydy, idlataan
        print(str(robot) + ": No balls found, idling")
        updateState(robot, RobotState.Idle)
        return
    if robot == robot_1_id: # Ykkosrobo jahtaa punaista
        key, ball = getClosestBall(tracked, robot_pose, robot_2_target_id)
        robot_1_target = target = ball
        robot_1_target_id = key
    if robot == robot_2_id: # Kakkosrobo jahtaa keltaista
        key, ball = getClosestBall(tracked, robot_pose, robot_1_target_id)
        robot_2_target = target = ball
        robot_2_target_id = key
    if target is not None:
        print(str(robot) + ": Starting chase...")
        """
        if robot == robot_1_id:
            path = astar(mask,
                (int(robot_pose[0] * 0.2), int(robot_pose[1] * 0.2)),
                (int(target.center[0] * 0.2), int(target.center[1] * 0.2))
            )
            path_x = [i[0] for i in path] 
            path_y = [i[1] for i in path]
            mask[path_x, path_y] = 125
            
            robot_1_path = []
            for index, item in enumerate(path):
                if(index%20 == 0):
                    tuple_item = (item[1]*5, item[0]*5)
                    robot_1_path.append(tuple_item)
            robot_1_path_current_node = 1
        if robot == robot_2_id:
            robot_2_path = astar(mask, robot_pose, target.center)
            robot_2_path_current_node = 1
        """
        updateState(robot, RobotState.ChaseTarget) # Kohde loytyy, lahdetaan peraan
    else:
        print(str(robot) + ": No suitable ball found, idling...")
        updateState(robot, RobotState.Idle) # Kohdetta ei loydy, idlataan

# Ajetaan pallon tyypista riippuen sen eteen tai taakse
def ChaseTarget(robot, tracked, robot_pose):
    global robot_1_target_id
    global robot_2_target_id
    global robot_1_id
    global robot_2_id
    #global robot_1_path
    #global robot_1_path_current_node
    #global robot_2_path
    #global robot_2_path_current_node

    id_number, target = getTarget(robot)
    print(str(robot) + ": Chasing target: " + str(id_number))

    # ASTAR _______ EPAKOMMENTOIDAAN JOS KAYTETAAN

    # Liikutaan pallon taakse
    #if robot == robot_1_id:
    #    print(str(robot) + ": Targeting node in " + str(robot_1_path[robot_1_path_current_node]))
    #    moveTowardsTarget(robot, robot_1_path[robot_1_path_current_node], robot_pose)
    #    if isNearTarget(robot_pose, robot_1_path[robot_1_path_current_node]): # Saavutettiin edellinen node, siirrytaan seuraavaan
    #        robot_1_path_current_node = robot_1_path_current_node + 1
    #        if robot_1_path_current_node >= len(robot_1_path): # Lahella palloa, siirrytaan elamassa eteenpain
    #            updateState(robot, RobotState.FindTarget)
    #if robot == robot_2_id:
    #    print(str(robot) + ": Targeting node in " + str(robot_2_path[robot_2_path_current_node]))
    #    moveTowardsTarget(robot, robot_2_path[robot_2_path_current_node], robot_pose)
    found = False
    for key in tracked.items():
        print("Ball available: " + str(key))
    for key, ball in tracked.items():
        if robot == robot_1_id:
            if str(key) == str(robot_1_target_id):
                found = True
        if robot == robot_2_id:
            if str(key) == str(robot_2_target_id):
                found = True
    if not found:
        print(str(robot) + ": Lost ball while chasing it, picking new target")
        updateState(robot, RobotState.FindTarget)
        return

    updateBallCoordinates(robot, tracked)
    moveTowardsTarget(robot, coordinatesForBall(target), robot_pose)
    # Jos ollaan riittavan lahella palloa, tahdataan siihen
    if isNearTarget(robot_pose, coordinatesForBall(target), 20):
        updateState(robot, RobotState.TurnTowardsTarget)


# Pusketaan pallo maaliin
def PushBallToGoal(robot, tracked, robot_pose):
    global SI1
    global SI2

    print(str(robot) + ": Pushing ball to goal")
    id_number, target = getTarget(robot)
    if target.color == -1:
        moveTowardsTarget(robot, own_goal_pose, robot_pose)
        if isNearTarget(robot_pose, own_goal_pose, 70):
            updateState(robot, RobotState.Idle)
    if target.color == 1:
        moveTowardsTarget(robot, opponent_goal_pose, robot_pose)
        if isNearTarget(robot_pose, opponent_goal_pose, 70):
            updateState(robot, RobotState.Idle)

    # Jos ollaan riittavan lahella, jyrataan pain
    #socket = None
    #if robot == robot_1_id:
    #    socket = SI1
    #if robot == robot_2_id:
    #    socket = SI2
    #socket.servo_forward()
    #if socket.get_distance(robot) < 20 * centimeter and isNearTarget(robot_pose, coordinatesForRobotBehindBall(target)):

    #    updateState(robot, RobotState.TurnTowardsTarget)
    
# Kaannytaan pallon suuntaan
def TurnTowardsTarget(robot, tracked, robot_pose):
    global robot_1_target
    global robot_2_target
    global SI1
    global SI2

    print(str(robot) + ": Rotating towards target")
    id_number, target = getTarget(robot)
    if target.color == -1:
        moveTowardsTarget(robot, opponent_goal_pose, robot_pose, 0)
        if isPointingTowards(robot_pose, opponent_goal_pose):
            updateState(robot, RobotState.PushBallToGoal)
    if target.color == 1:
        moveTowardsTarget(robot, own_goal_pose, robot_pose, 0)
        if isPointingTowards(robot_pose, own_goal_pose):
            updateState(robot, RobotState.PushBallToGoal)


# Evaluoi robotin tilaa, pyorittaa tilakonetta
def evaluateRobotState(robot, ball_positions, robot_positions):
    global robot_1_state
    global robot_2_state
    robotStates = {
        0: Idle,
        1: FindTarget,
        2: ChaseTarget,
        3: PushBallToGoal,
        4: TurnTowardsTarget,
    }
    robot_pose = (0.0, 0.0, 0.0)
    pose_found = False
    for key, pos in robot_positions.items():
        if key == robot:
            robot_pose = (pos[0], pos[1], pos[2])
            pose_found = True
    if not pose_found:
        print(str(robot) + ": No robot found, idling")
        updateState(robot, RobotState.Idle)
    if robot == robot_1_id:
        print(str(robot) + ": Running state " + str(robot_1_state))
        robotStates[robot_1_state](robot, ball_positions, robot_pose)
    if robot == robot_2_id:
        print(str(robot) + ": Running state " + str(robot_2_state))
        robotStates[robot_2_state](robot, ball_positions, robot_pose)

# Vaihdetaan tilakoneen tilaa
def updateState(robot, newState):
    global robot_1_state
    global robot_2_state
    if robot == robot_1_id:
        robot_1_state = newState
        print(str(robot) + " changes to state " + str(robot_1_state))
    if robot == robot_2_id:
        robot_2_state = newState
        print(str(robot) + " changes to state " + str(robot_2_state))



# HELPER METHODS

# Luetaan kohde
def getTarget(robot):
    global robot_1_target
    global robot_2_target
    global robot_1_target_id
    global robot_2_target_id
    if robot == robot_1_id:
        return robot_1_target_id, robot_1_target
    if robot == robot_2_id:
        return robot_2_target_id, robot_2_target

"""
Return coordinates behind the ball based on the color of the ball and the position of the goal

Input: 
    a ball (x, y, ballType)
"""
def coordinatesForRobotBehindBall(ball,correction):
    global opponent_goal_pose
    global own_goal_pose

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

    pos_x = position_behind_ball[0]
    pos_y = position_behind_ball[1]
    if pos_x <=  0:
        pos_x = correction
    if pos_y <= 0:
        pos_y = correction
    if pos_x >= 972:
        pos_x = 972-correction
    if pos_y >= 972:
        pos_y = 972-correction
    return (pos_x,pos_y)


# Sama kuin ylempi mutta yrittaa ajaa suoraan palloon
def coordinatesForBall(ball):
    global opponent_goal_pose
    global own_goal_pose

    x,y = ball.center
    color = ball.color
    position_behind_ball = np.array([x, y])
    
    return position_behind_ball

def isNearTarget(robot_pose, target, distanceInCentimeters):
    dist = distance.euclidean((target[0], target[1]), (robot_pose[0], robot_pose[1]))
    if dist < centimeter * distanceInCentimeters:
        return True
    return False

#pi to pi
def normalize_angle(angle):
    if  angle < -2.0*math.pi or angle > 2.0*math.pi:
        n   = math.floor(angle/(2.0*math.pi))
        angle = angle - n*(2.0*math.pi)
    

    if angle > math.pi:
        angle = angle - (2.0*math.pi)

    if angle < -math.pi:
        angle = angle + (2.0*math.pi)

    return angle

def isPointingTowards(robot_pose, target_pose):
    DEG2RAD = math.pi/180.0
    RAD2DEG = 180.0/math.pi
    goal_theta = math.atan2(target_pose[0]-robot_pose[0], target_pose[1]-robot_pose[1])
    theta_error = normalize_angle(robot_pose[2] - goal_theta)
    if abs(theta_error) < DEG2RAD * 10.0:
        return True
    return False

def getClosestBall(tracked, robot_pose, ignore_ball):
    chosenBall = None
    chosenBallId = -1
    shortestDistance = 100000
    for key, ball in tracked.items():
        if key is not ignore_ball:
            if distance.euclidean(ball.center, own_goal_pose) > 60 * centimeter and distance.euclidean(ball.center, opponent_goal_pose) > 60 * centimeter
                dist = distance.euclidean(ball.center, (robot_pose[0], robot_pose[1]))
                if (dist < shortestDistance):
                    shortestDistance = dist
                    chosenBall = ball
                    chosenBallId = key
    return chosenBallId, chosenBall

def updateBallCoordinates(robot, tracked):
    global robot_1_id
    global robot_1_target
    global robot_2_id
    global robot_2_target
    for key, ball in tracked.items():
        if robot == robot_1_id:
            if key == robot_1_target_id:
                robot_1_target = ball
        if robot == robot_2_id:
            if key == robot_2_target_id:
                robot_2_target = ball

# ROBOTIN LIIKKUMINEN

def ramForward(robot):
    print("Nyt mennaan")
    # AJETAAN LUJAA PaIN

def moveTowardsTarget(robot, target_pose, robot_pose, speed = 0.4):
    global SI1
    global SI2

    '''target_x = target_pose[0]
    target_y = target_pose[1]
    robot_x = robot_pose[0]
    robot_y = robot_pose[1]
    robot_yaw = robot_pose[2]

    # ohjauskomento
    r_com = 0.0
    l_com = 0.0
    deadzone = 90.0
    brakezone = 50.0
    print("r_com: "+str(r_com))
    print("l_com: "+str(l_com))
    r_com, l_com = drive_commands(
        target_x, target_y, robot_x, robot_y, robot_yaw)
    r_com = 70.0*r_com #255*r_com
    l_com = 70.0*l_com #255*l_com
    print("r_com: "+str(r_com))
    print("l_com: "+str(l_com))

    """
    if abs(r_com) < brakezone:
        r_com = 0.0
    elif abs(r_com) < deadzone:
        if r_com < 0.0:
            r_com = r_com - deadzone
        if r_com > 0.0:
            r_com = r_com + deadzone
    if abs(l_com) < brakezone:
        l_com = 0.0
    elif abs(l_com) < deadzone:
        if l_com < 0.0:
            l_com = l_com - deadzone
        if r_com > 0.0:
            l_com = l_com + deadzone
    """
    print("r_com: "+str(r_com))
    print("l_com: "+str(l_com))
    '''

    ball_x = target_pose[0]
    ball_y = target_pose[1]
    robot_x = robot_pose[0]
    robot_y = robot_pose[1]
    robot_yaw = robot_pose[2]

    # ohjauskomento
    r_com = 0.0
    l_com = 0.0
    deadzone = 90
    brakezone = 50

    r_com, l_com = drive_commands(
        ball_x, ball_y, robot_x, robot_y, robot_yaw, speed)
    r_com = 170*r_com #255*r_com
    l_com = 170*l_com #255*l_com
    #print(l_com,r_com)

    # ohjauskomento sokettiin
    if robot == robot_1_id:
        SI1.send_command(r_com, l_com)
    if robot == robot_2_id:
        SI2.send_command(r_com, l_com)

if __name__ == "__main__":
    main()
