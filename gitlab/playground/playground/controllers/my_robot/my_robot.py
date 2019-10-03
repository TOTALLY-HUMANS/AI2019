import time
import sys
from websocket import create_connection
from invader_bot import InvaderBot

if(len(sys.argv) < 2):
    raise Exception("Add script argument (robot_id) to the robot!")
robot_id = sys.argv[1]

# make sure that commucation server is up and running before connecting
# (lazy way to do it, just to use sleep...)
time.sleep(1)
ws = create_connection("ws://localhost:9000")
ws.send("NEW_ROBOT:" + str(robot_id))

bot = InvaderBot()
bot.setup()

# main loop
while bot.step():

    message = ws.recv()
    (left, right) = message.split("|")
    left = float(left)
    right = float(right)
    #print(left, right)
    bot.set_motors(left, right)
    
    print (bot.get_position())
    print (bot.get_balls())
