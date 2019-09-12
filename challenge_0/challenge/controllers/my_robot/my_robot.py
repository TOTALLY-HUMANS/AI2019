# EDIT THIS FILE

# InvaderBot is a simple helper class which has methods to control the robot.
# It offers an interface over Webots' default motor controller.
# You can use it, modify it or you can also use Webots' own commands.
# Using the helper class is recommended because it will later help you to
# transfer your code into a physical robot, which is running on Raspberry Pi.

from invader_bot import InvaderBot

bot = InvaderBot()
bot.setup()

# main loop
while bot.step():

    # get current time
    time = bot.get_time()

    if time < 1.0:
        bot.set_motors(1, 1)
    else:
        bot.set_motors(-0.75, 0.75)
