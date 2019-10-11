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
#from video_capture import VideoCapture

from ball_detector import BallDetector
#from aruco_detector import ArucoDetector
#from euclidian_tracker import EuclidianTracker

from my_robot import drive_commands
from socket_interface import socketInterface


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
    #cap = AvVideoCapture(config["url"])
    #cap = VideoCapture(url)
    cap = cv2.VideoCapture('videos/Balls1.ts')
    print("Initializing ball detector.")
    ball_detector = BallDetector(config["ball_detector"], debug=True)
    try:
        while 1:
            ret, img = cap.read()

            if ret:

                img = downscale_image(img, config["downscale_p"])

                balls = ball_detector.detect_balls(img)  

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
