#!/usr/bin/env python
import math
import cv2
import numpy as np

from ball import Ball,Color

def nothing(x):
    return

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


class BallDetector:

    def __init__(self, config, debug=False):
        self.yellowLowTres = np.array(config["yellow_low"])
        self.yellowHighTres = np.array(config["yellow_high"])
        self.pinkLowTres = np.array(config["pink_low"])
        self.pinkHighTres = np.array(config["pink_high"])
        self.ballSizeRange = np.array(config["radius_range"])
        self.debug = debug
        self.fast = cv2.FastFeatureDetector_create()
        if debug:
            self.create_debug_windows()
            self.create_trackbars()

    def detect_blobs(self,img, value):
        kps = self.fast.detect(img,None)
        kps =  merge_keypoints(kps,35)
        return [ kp.pt for kp in kps]

    def detect_balls(self, img):
        yellow_mask = self.maskFrame(img, self.yellowLowTres, self.yellowHighTres)
        yellow_kps = self.detect_blobs(yellow_mask,Color.YELLOW)
        yellow = self.detect_balls_in_keypoints(yellow_mask,yellow_kps,1,50)
        #yellow = self.hough_detect_balls(yellow_mask, 1)
        
        pink_mask = self.maskFrame(img, self.pinkLowTres, self.pinkHighTres)
        pink_kps = self.detect_blobs(pink_mask, Color.PINK)
        pink = self.detect_balls_in_keypoints(pink_mask,pink_kps,-1,50)
        #pink = self.hough_detect_balls(pink_mask, -1)

        balls = [*yellow, *pink]
        and_mask = cv2.bitwise_or(yellow_mask,pink_mask)
        #balls = pink
        if self.debug:
            self.debug_update(img,yellow_mask,pink_mask, balls)
        
        return balls, and_mask
    
    def detect_balls_in_keypoints(self,img, kps, value, side):
        half = side/2
        ret = []
        #Use keypoints to efficiently search for balls
        for kp in kps:
            x = kp[0]
            y = kp[1]
            x1 = int(x-half)
            x2 = int(x+half)
            y1 = int(y-half)
            y2 = int(y+half)
            area = img[y1:y2,x1:x2]
            balls = self.hough_detect_balls(area, value)
            #Take only one ball from each keypoint
            if balls:
                ball = balls[0]
                w_x = ball[0]+x1
                w_y = ball[1]+y1
                r = ball[2]
                ball_obj = Ball((w_x,w_y),r,(0,0),value)
                ret.append(ball_obj)
        return ret

    def hough_detect_balls(self, img, value):
        #cv2.imshow("masked"+str(value), img)
        if img.shape[0] == 0 or img.shape[1] == 0:
            return []
        #print(img.shape)
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1,
                                   img.shape[0]/64, param1=100, param2=8, minRadius=self.ballSizeRange[0], maxRadius=self.ballSizeRange[1])
        if circles is None:
            #print("No circles found")
            return []
        circles = np.round(circles[0, :]).astype("int")
        r = [tuple(x)+(value,) for x in circles]
        return r

    def maskFrame(self, frame, low_b, high_b):
        """
        Perform basic preprocessing to create a mask that can be overlayed over the
        image. This will dramatically reduce the search space for more complicated
        operations further down the pipeline, speeding up computation.

        :param frame: The imput image
        :return: A binary mask corresponding to parts of the image that have
        similar hue, saturation, and value levels as the objects to be detected
        """
        # Blur the image to reduce high frequency noise
        mask = cv2.GaussianBlur(frame,(3,3),0);
        
        #mask = cv2.medianBlur(frame, 3)
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        
        mask = frame

        # Convert the colorspace to HSV (Hue, Saturation, Value)
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)
        # Mask frame  to contain wanted colors.
        mask = cv2.inRange(mask, low_b, high_b)
        # Remove more noise
        #mask = cv2.fastNlMeansDenoising(mask,None,10,7,21)
        return mask

    def debug_update(self, img, yellow_mask, pink_mask, balls):
        self.draw_debug(img,yellow_mask,pink_mask,balls)
        self.update_trackbars()
    
    def draw_debug(self,img, yellow_mask,pink_mask,balls):
        for ball in balls:
            color = (127, 0, 255)
            if ball.color == Color.YELLOW:
                color = (0, 179, 255)
            print(ball)
            cv2.circle(img, ball.center, ball.radius, color, 2)
            cv2.circle(img, ball.center, 2, color, 2)
        cv2.imshow('yellow', yellow_mask)
        cv2.imshow('pink', pink_mask)
        cv2.imshow('debug', img)
        key = cv2.waitKey(1)

    def create_debug_windows(self):
        cv2.namedWindow('debug')
        cv2.namedWindow('yellow')
        cv2.namedWindow('pink')

    
    def create_trackbars(self):
        #Low yellow hsv trackers
        cv2.createTrackbar('H_low', 'yellow', self.yellowLowTres[0], 180, nothing)
        cv2.createTrackbar('S_low', 'yellow', self.yellowLowTres[1], 255, nothing)
        cv2.createTrackbar('V_low', 'yellow', self.yellowLowTres[2], 255, nothing)
        #Upper yellow hsv trackers
        cv2.createTrackbar('H_high', 'yellow', self.yellowHighTres[0], 180, nothing)
        cv2.createTrackbar('S_high', 'yellow', self.yellowHighTres[1], 255, nothing)
        cv2.createTrackbar('V_high', 'yellow', self.yellowHighTres[2], 255, nothing)
        #Low pink hsv trackers
        cv2.createTrackbar('H_low', 'pink', self.pinkLowTres[0], 180, nothing)
        cv2.createTrackbar('S_low', 'pink', self.pinkLowTres[1], 255, nothing)
        cv2.createTrackbar('V_low', 'pink', self.pinkLowTres[2], 255, nothing)
        #High pink hsv trackers
        cv2.createTrackbar('H_high', 'pink', self.pinkHighTres[0], 180, nothing)
        cv2.createTrackbar('S_high', 'pink', self.pinkHighTres[1], 255, nothing)
        cv2.createTrackbar('V_high', 'pink', self.pinkHighTres[2], 255, nothing)
        #Ball size trackers
        cv2.createTrackbar('R_low', 'debug', self.ballSizeRange[0], 200, nothing)
        cv2.createTrackbar('R_high', 'debug', self.ballSizeRange[1], 200, nothing)
    
    def update_trackbars(self):
        #Update yellow low treshold
        self.yellowLowTres[0] = cv2.getTrackbarPos('H_low', 'yellow')
        self.yellowLowTres[1] = cv2.getTrackbarPos('S_low', 'yellow')
        self.yellowLowTres[2] = cv2.getTrackbarPos('V_low', 'yellow')
        #Update yellow high treshold
        self.yellowHighTres[0] = cv2.getTrackbarPos('H_high', 'yellow')
        self.yellowHighTres[1] = cv2.getTrackbarPos('S_high', 'yellow')
        self.yellowHighTres[2] = cv2.getTrackbarPos('V_high', 'yellow')
        #Update pink low treshold
        self.pinkLowTres[0] = cv2.getTrackbarPos('H_low', 'pink')
        self.pinkLowTres[1] = cv2.getTrackbarPos('S_low', 'pink')
        self.pinkLowTres[2] = cv2.getTrackbarPos('V_low', 'pink')
        #Update pink high treshold
        self.pinkHighTres[0] = cv2.getTrackbarPos('H_high', 'pink')
        self.pinkHighTres[1] = cv2.getTrackbarPos('S_high', 'pink')
        self.pinkHighTres[2] = cv2.getTrackbarPos('V_high', 'pink')
        #Update ball size range
        self.ballSizeRange[0] = cv2.getTrackbarPos('R_low', 'debug')
        self.ballSizeRange[1] = cv2.getTrackbarPos('R_high', 'debug')




