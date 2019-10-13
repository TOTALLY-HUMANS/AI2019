#!/usr/bin/env python
import time
import math
import json
# import imutils
import cv2
import av
import numpy as np

#from scipy.spatial import distance

from enum import Enum

from av_video_capture import AvVideoCapture
from video_capture import VideoCapture

from ball_detector import BallDetector
from aruco_detector import ArucoDetector
#from euclidian_tracker import EuclidianTracker

from my_robot import drive_commands
from socket_interface import socketInterface

from astar import astar

url= "udp://224.0.0.0:1234"
def downscale_image(img, scale_percent):
    width = int(img.shape[1] * scale_percent / 100)
    height = int(img.shape[0] * scale_percent / 100)
    dim = (width, height)

    return cv2.resize(img, dim, interpolation=cv2.INTER_AREA)


def main():
    print("Loading configuration.")
    with open('config.json') as json_data:
        config = json.load(json_data)

    print("Connecting to camera.")
    #cap = AvVideoCapture(url)
    #cap = VideoCapture(url)

    cap = cv2.VideoCapture("videos/ArucoVideo.ts")

    print("Initializing ball detector.")
    ball_detector = BallDetector(config["ball_detector"], debug=False)
    aruco_detector = ArucoDetector()
    print(config)
    cm_in_pixels = (1080*(config["downscale_p"]/100))/config["arena_side"]
    robot_max_width = 40*cm_in_pixels
    robot_half_width = robot_max_width*0.5
    print(cm_in_pixels)
    try:
        while 1:
            ret, img = cap.read()
            

            if ret:

                img = downscale_image(img, config["downscale_p"]) 
                # 972 x 972

                balls, mask = ball_detector.detect_balls(img)  
                mask[mask == 255] = 150
                corners, ids = aruco_detector.get_arucos(img)
                positions = aruco_detector.get_positions(corners, ids)
                #print(mask.max())
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
                
                center_x = 500
                center_y = 450
                theta = 7.1
                c,s = np.cos(theta), np.sin(theta)
                t = np.array([center_x,center_y])
                rot_mat = np.array((c, -s),(s, c))
                """
                for i2 in range(-150,150):
                    for j2 in range(-150,150):
                        trans_i = i2*c +s*j2
                        trans_j = -i2*s +c*j2
                        mask[int(trans_i+center_y),int(trans_j+center_x)] = 150
                """
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))  
                mask = cv2.dilate(mask, kernel, iterations=10)
                mask = cv2.resize(mask, (int(972*0.2), int(972*0.2)))
                #mask = downscale_image(img, 20)
                #print(mask.shape)

                # find path
                start_time = time.time()
                path = astar(mask, (0, 0), (190,190))
                elapsed_time = time.time() - start_time
                print("astar time:" + str(elapsed_time))
                path_x = [i[0] for i in path] 
                path_y = [i[1] for i in path]
                mask[path_x, path_y] = 125
                
                scaled_path = []
                for index, item in enumerate(path):
                    if(index%20 == 0):
                        tuple_item = (item[1]*5, item[0]*5)
                        scaled_path.append(tuple_item)
                        cv2.circle(img, tuple_item, 2, (0,0,225))

                #print(scaled_path)
                cv2.imshow('orig', img)
                #cv2.imshow('mask_test', mask)
                key = cv2.waitKey(1)

    except:
        raise
    finally:
        ball_detector_config = config["ball_detector"]
        ball_detector_config["yellow_low"] = ball_detector.yellowLowTres.tolist()
        ball_detector_config["yellow_high"] = ball_detector.yellowHighTres.tolist()
        ball_detector_config["pink_low"] = ball_detector.pinkLowTres.tolist()
        ball_detector_config["pink_high"] = ball_detector.pinkHighTres.tolist()
        ball_detector_config["radius_range"] = ball_detector.ballSizeRange.tolist()
        config["ball_detector"] = ball_detector_config
        with open('config.json', 'w') as f:
            json.dump(config, f, ensure_ascii=False)
            print("Saved config")
        print("closed.")


if __name__ == "__main__":
    main()
