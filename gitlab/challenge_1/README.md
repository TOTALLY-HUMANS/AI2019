## #1 POWER UP

![Challenge 1](img/webots_1.png "Webots Challenge 1")


### TARGET

Push all balls into the goal

### DESCRIPTION

The purpose of this challenge is to get you aquanted with the core
gameplay rules. You will start in an arena which simulates the game
arena for the main event. There are 7 balls scattered about the 
arena. Your task is to push all 7 balls into the blue goal.

Driving around randomly won't help you, since a single ball going through 
the red goal will result in a failure.

You can freely modify the files under the folder `challenge/controllers/my_robot/`.
All other files will be replaced by the original ones.

To open the challenge, open the file `challenge/arena/challenge1.wbt` with Webots.

### OTHER

InvaderBot is a simple helper class which has methods to control the robot.
It offers an interface over Webots' default motor controller.
You can use it, modify it or you can also use Webots' own commands.
Using the helper class is recommended because it will later help you to
transfer your code into a physical robot, which is running on Raspberry Pi.

If you have perfomance issues, lower Webot's rendering settings. You can also
try to delete a Webot's object called "DEF \_lights GROUP" to improve perfomance.
