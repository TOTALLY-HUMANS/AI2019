import time
import threading

import numpy as np

import OpenCVClient as client
from controller import Robot
from OpenCVServer import OpenCVServer

TIME_STEP = 64
print("Starting, getting camera...")
bot = Robot()

# Get the camera device, enable it, and store its width and height
camera = bot.getCamera("camera");
camera.enable(TIME_STEP);
width = camera.getWidth();
height = camera.getHeight();
print("Done.")

def start_cv_client():
    time.sleep(10)
    while True:
        try:
            client.main()
            break
        except:
            print("Restarting OpenCVClient...")
            time.sleep(3)

threading.Thread(target=start_cv_client, args=()).start()
        
# Start OpenCVServer
print("Init OpenCVServer...")
cv = OpenCVServer()
print("Done.")

def get_and_send_img():
    """Get image from Webots and send it over a socket from OpenCVServer to OpenCVClient."""
    img = camera.getImageArray()

    img = [[[point[2], point[1], point[0]] for point in row] for row in img] # Remove alpha channel and convert RGB -> BGR
    img = np.array(img, dtype=np.uint8).tobytes()

    threading.Thread(target=cv.send, args=(img,)).start()

# Main loop
time.sleep(5)
time_counter = time.time()
while bot.step(TIME_STEP) != -1:
    if time.time() - time_counter > 5:
        get_and_send_img()
        time_counter = time.time()
