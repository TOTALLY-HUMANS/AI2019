from collections import OrderedDict
from scipy.spatial import distance as dist
import numpy as np


# Adapted from https://www.pyimagesearch.com/2018/07/23/simple-object-tracking-with-opencv/

class EuclidianTracker():

    def __init__(self, maxDisappeared=50):
        self.nextObjectID = 0
        self.objects = OrderedDict()
        self.disappeared = OrderedDict()
        # max frames to stay disappeared
        self.maxDisappeared = maxDisappeared

    def register(self, obj):
        self.objects[self.nextObjectID] = obj
        self.disappeared[self.nextObjectID] = 0
        self.nextObjectID += 1

    def deregister(self, objectID):
        del self.objects[objectID]
        del self.disappeared[objectID]

    def update(self, balls):
        # Update disappeared object frames
        if len(balls) == 0:
            for objectID in list(self.disappeared.keys()):
                self.disappeared[objectID] += 1
                if self.disappeared[objectID] > self.maxDisappeared:
                    self.deregister(objectID)
            return self.objects
        # Register all objects if there are none
        if len(self.objects) == 0:
            for i in range(0, len(balls)):
                self.register(balls[i])
        else:
            objectIDs = list(self.objects.keys())
            objectCentroids = np.array([(obj[0], obj[1])for obj in self.objects.values()])
            inputCentroids = np.array([(obj[0], obj[1]) for obj in balls])

            D = dist.cdist(objectCentroids, inputCentroids)
            rows = D.min(axis=1).argsort()
            cols = D.argmin(axis=1)[rows]

            usedRows = set()
            usedCols = set()

            for row, col in zip(rows, cols):
                if row in usedRows or col in usedCols:
                    continue
                
                objectID = objectIDs[row]
                self.objects[objectID] = balls[col]
                self.disappeared[objectID] = 0

                usedRows.add(row)
                usedCols.add(col)

                unusedRows = set(range(0, D.shape[0])).difference(usedRows)
                unusedCols = set(range(0, D.shape[1])).difference(usedCols)

            if D.shape[0] >= D.shape[1]:
                for row in unusedRows:
                    objectID = objectIDs[row]
                    self.disappeared[objectID] += 1
                    if self.disappeared[objectID] > self.maxDisappeared:
                        self.deregister(objectID)
            else:
                for col in unusedCols:
                    self.register(balls[col])
        return self.objects
