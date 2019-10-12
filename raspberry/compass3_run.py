import compass3
import time
import math

compass3.wake()

try:
    while True:
        print(compass3.getHeading())
        time.sleep(1)
except KeyboardInterrupt:
    compass3.sleep()

