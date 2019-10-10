
from enum import Enum

class Color(Enum):
    PINK = -1
    YELLOW = 1

class Ball:
    def __init__(self, center, radius, speed, color):
        self.center = center
        self.radius = radius
        self.speed = speed
        self.color = color
    
    def __repr__(self):
        return "Ball({},{},{},{})".format(self.center,self.radius,self.speed,self.color)
    def __str__(self):
        return "Center: {} Radius: {} Speed: {} Color: {}".format(self.center,self.radius,self.speed,self.color)
