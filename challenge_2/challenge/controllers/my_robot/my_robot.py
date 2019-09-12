"""EDIT THIS FILE

InvaderBot is a simple helper class which has methods to control the robot.
It offers an interface over Webots' default motor controller.
You can use it, modify it or you can also use Webots' own commands.
Using the helper class is recommended because it will later help you to
transfer your code into a physical robot, which is running on Raspberry Pi."""

from invader_bot import InvaderBot
from cv_connector import OpenCVConnector
import time

bot = InvaderBot()
bot.setup()

kb = bot.robot.getKeyboard()
kb.enable(bot.timestep)

print("Starting OpenCVConnector...")
connector = OpenCVConnector()
connector.initConnection()
print("Started.")

# main loop
print("Starting main loop...")
while bot.step() != -1:

    balls = connector.getBallCoordinates()
    
    ball0 = balls[0] if len(balls) > 0 else (0, 0)

    #print("Heading to balls[0] at " + str(ball0))

    bot.set_motors(1,-1)

print("my_robot is kill")
