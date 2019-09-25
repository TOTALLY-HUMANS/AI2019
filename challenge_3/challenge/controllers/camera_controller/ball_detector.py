#!/usr/bin/env python
import math
import cv2
import numpy as np


class BallDetector:

    def __init__(self, yellowLowTres, yellowHighTres, pinkLowTres, pinkHighTres, ballSizeRange=(15, 30), debug=False):
        self.yellowLowTres = yellowLowTres
        self.yellowHighTres = yellowHighTres
        self.pinkLowTres = pinkLowTres
        self.pinkHighTres = pinkHighTres
        self.ballSizeRange = ballSizeRange
        self.debug = debug

    def detect_balls(self, img):
        yellow = self.hough_detect_balls(
            self.maskFrame(img, self.yellowLowTres, self.yellowHighTres), 1)
        pink = self.hough_detect_balls(
            self.maskFrame(img, self.pinkLowTres, self.pinkHighTres), -1)
        balls = [*yellow, *pink]
        if self.debug:
            self.draw_debug(img, balls)
        return balls

    def hough_detect_balls(self, img, value):
        cv2.imshow("masked"+str(value), img)
        circles = cv2.HoughCircles(img, cv2.HOUGH_GRADIENT, 1,
                                   img.shape[0]/64, param1=200, param2=10, minRadius=self.ballSizeRange[0], maxRadius=self.ballSizeRange[1])
        if circles is None:
            print("No circles found")
            return []
        print(circles)
        circles = np.round(circles[0, :]).astype("int")
        print(circles)
        r = [tuple(x)+(value,) for x in circles]
        print(r)
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


        # Convert the colorspace to HSV (Hue, Saturation, Value)
        mask = cv2.cvtColor(mask, cv2.COLOR_BGR2HSV)
        # Mask frame  to contain wanted colors.
        mask = cv2.inRange(mask, low_b, high_b)
        # Remove more noise
        
        return mask

    def draw_debug(self, img, balls):
        for ball in balls:
            x, y, r, v = ball
            color = (127, 0, 255)
            if v == 1:
                color = (0, 179, 255)
            cv2.circle(img, (x, y), r, color, 2)
        cv2.imshow('DEBUG', img)
        # print(balls)
        key = cv2.waitKey(1)
