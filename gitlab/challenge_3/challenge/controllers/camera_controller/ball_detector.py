#!/usr/bin/env python
import math
import cv2
import numpy as np

def nothing(x):
    return

class BallDetector:

    def __init__(self, yellowLowTres, yellowHighTres, pinkLowTres, pinkHighTres, ballSizeRange=(15, 30), debug=False):
        self.yellowLowTres = yellowLowTres
        self.yellowHighTres = yellowHighTres
        self.pinkLowTres = pinkLowTres
        self.pinkHighTres = pinkHighTres
        self.ballSizeRange = ballSizeRange
        self.debug = debug
        if debug:
            self.create_debug_windows()
            self.create_trackbars()
            
            




    def detect_balls(self, img):
        yellow_mask = self.maskFrame(img, self.yellowLowTres, self.yellowHighTres)
        yellow = self.hough_detect_balls(yellow_mask, 1)

        pink_mask = self.maskFrame(img, self.pinkLowTres, self.pinkHighTres)
        pink = self.hough_detect_balls(pink_mask, -1)

        balls = [*yellow, *pink]
        if self.debug:
            self.debug_update(img,yellow_mask,pink_mask, balls)
        return balls

    def hough_detect_balls(self, img, value):
        #cv2.imshow("masked"+str(value), img)
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1,
                                   img.shape[0]/64, param1=200, param2=10, minRadius=self.ballSizeRange[0], maxRadius=self.ballSizeRange[1])
        if circles is None:
            print("No circles found")
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
        #frame = cv2.GaussianBlur(frame,(7,7),0);
        
        mask = cv2.medianBlur(frame, 3)
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
            x, y, r, v = ball
            color = (127, 0, 255)
            if v == 1:
                color = (0, 179, 255)
            cv2.circle(img, (x, y), r, color, 2)
        cv2.imshow('yellow', yellow_mask)
        cv2.imshow('pink', pink_mask)
        cv2.imshow('debug', img)
        cv2.imwrite('test.png',img)
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




