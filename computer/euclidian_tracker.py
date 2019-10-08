from collections import OrderedDict
import numpy as np

#Adapted from https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/

class EuclidianTracker():
    def __init__(self, maxDisappeared=50):
        self.nextObjectID = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
        #max frames to stay disappeared
        self.maxDisappeared = maxDisappeared
    
    def register(self, obj):
        self.objects[self.nextObjectID] = obj
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID +=1
    
    #def deregister(self, objectID):
        
