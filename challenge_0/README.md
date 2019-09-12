## #0 BOTS ON THE RUN

![Challenge 0](img/webots_0.png "Webots Challenge 0")


### TARGET

Gather all coins with your robot.

### DESCRIPTION

The purpose of this challenge is to teach you how to use Webots.
Your task is simply to make a script that moves the bot.
The bot should gather all coins. The coins are static and always
in the same position.

You do not have access to the coordinates of your robot
because your robot does not have any sensors yet.
In the tournament, there will be a camera streaming the arena from the ceiling.
The position of the objects can be calculated by using image recognizing.
In later challenges, image recognition will be simulated, but not yet.

You can freely modify the files under the folder ```challenge/controllers/my_robot/```.
Changes to all other files will be ignored by the grading system even if you try to submit them.

Once you want to submit the challenge for grading, move or copy the files you've modified from ```challenge/controllers/my_robot``` to the ```submission``` folder and commit the changes. The CI will take care of the rest.

To open the challenge, open the file ```challenge/arena/challenge0.wbt``` with Webots.

### NOTES

InvaderBot is a simple helper class which has methods to control the robot.
It offers an interface over Webots' default motor controller.
You can use it, modify it or you can also use Webots' own commands.
Using the helper class is recommended because it will later help you to
transfer your code into a physical robot, which is running on Raspberry Pi.

If you have perfomance issues, lower Webots' rendering settings. You can also
try to delete a Webots' object called "DEF \_lights GROUP" to improve perfomance.