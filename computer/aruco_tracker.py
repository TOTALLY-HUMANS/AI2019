class ArucoTracker():

    def __init__(self):
        self.objects = {}

    def update(self, objects):
        if objects:
            for o in objects:
                print(o[3][0])
                self.objects[o[3][0]] = o
        return self.objects


