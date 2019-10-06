import numpy as np
import cv2
import math
from cv2 import aruco

# Install numpy and opencv including aruco with pip
# -> pip install numpy
# -> pip install opencv-contrib-python
# https://stackoverflow.com/questions/45972357/python-opencv-aruco-no-module-named-cv2-aruco


class ArucoDetector:

    def __init__(self):
        # Default parameters are enough
        self.parameters = aruco.DetectorParameters_create()
        # Using aruco 4x4 with ids 0 - 49
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    
    def get_arucos(self, img):
        corners, ids, rejectedImgPoints = aruco.detectMarkers(
            img, self.aruco_dict, parameters=self.parameters)
        return (corners, ids)

    def get_positions(self, corners, ids):
        positions = []
        if ids is not None:
            for i in range(len(corners)):
                # https://gitlab.com/artificial-invaders-2019/ai-sample-stack
                c = corners[i][0]
                x = 0.25 * (c[0][0] + c[1][0] + c[2][0] + c[3][0])
                z = 0.25 * (c[0][1] + c[1][1] + c[2][1] + c[3][1])
                theta = math.atan2(c[0][1] - c[1][1], c[0][0] - c[1][0])
                pos = (x, z, theta, ids[i])
                positions.append(pos)
        return positions
