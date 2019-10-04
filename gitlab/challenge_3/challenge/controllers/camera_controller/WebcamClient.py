#!/usr/bin/env python
import time
import math
import imutils
import cv2
import numpy as np
from ball_detector import BallDetector

# Greenish yellow hsv (25,100,150) ~ (35, 255, 255)
yellow_low = np.array([25, 50, 100])
yellow_high = np.array([40, 255, 255])
# Pink (150, 100, 150)  ~ (160, 255, 255)
pink_low = np.array([150, 50, 80])
pink_high = np.array([195, 255, 255])

radius_range = [13, 30]


def nothing(x):
    pass


def main():
    print("Connecting to camera")
    camera = cv2.VideoCapture(0)

    print("Initializing feature detector.")

    # fast = cv2.FastFeatureDetector_create()

    detector = BallDetector(yellow_low, yellow_high, pink_low,
                            pink_high, ballSizeRange=radius_range, debug=True)

    try:
        print("trying")
        while 1:
            print("Listening for image...")
            
            #cv2.createTrackbar('B', 'debug', 0, 255, nothing)

            ret, img = camera.read()
            balls = detector.detect_balls(img)
            print(balls)
            # time.sleep(1)
    except:
        raise
    finally:
        print("closed.")


if __name__ == "__main__":
    main()
