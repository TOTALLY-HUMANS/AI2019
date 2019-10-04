from controller import Supervisor
import math

class Botsy():

    # Initalize the motors.
    def setup(self):

        self.robot = Supervisor()
        self.motor_left = self.robot.getMotor("motor_left")
        self.motor_right = self.robot.getMotor("motor_right")

        self.timestep = int(self.robot.getBasicTimeStep())

    # Do one update step. Calls Webots' robot.step().
    # Return True while simulation is running.
    # Return False if simulation is ended
    def step(self):
        return (self.robot.step(self.timestep) != -1)

    # Set the velocity of the motor [-1, 1].
    def set_motor(self, motor, velocity):
        mult = 1 if velocity > 0 else -1
        motor.setPosition(mult*float('+inf'))
        motor.setVelocity(velocity*motor.getMaxVelocity())

    # Set the velocity of left and right motors [-1, 1].
    def set_motors(self, left, right):
        self.set_left_motor(left)
        self.set_right_motor(right)

    # Set the velocity of left and right motors [-1, 1].
    def turn_right(self, speed):
        self.set_left_motor(speed)
        self.set_right_motor(-speed)
    def turn_left(self, speed):
        self.set_left_motor(-speed)
        self.set_right_motor(speed)

    # Set the velocity of left motors [-1, 1].
    def set_left_motor(self, velocity):
        self.set_motor(self.motor_left, velocity)

    # Set the velocity of right motors [-1, 1].
    def set_right_motor(self, velocity):
        self.set_motor(self.motor_right, velocity)

    # Returns the current simulation time in seconds
    def get_time(self):
        return self.robot.getTime()

    # Returns the position of the robot in [x, z, angle] format
    # The coordinate system is as follows (top-down view)
    #  .-------------------->x
    #  |\  (angle)
    #  | \
    #  |  \
    #  |   \
    #  |    \
    #  |     \
    #  |
    #  V
    #  z
    #
    def get_position(self):
        subject = self.robot.getSelf()
        position = subject.getPosition()
        orientation = subject.getOrientation()
        orientation = math.atan2(orientation[0], orientation[2])
        return (position[0],position[2],orientation)

    # Returns the position of the green balls
    def get_green_balls(self):
        balls = []
        balls_root = self.robot.getFromDef("_balls").getField("children")
	
        green_balls = balls_root.getMFNode(1).getField("children")
        
        for i in range(green_balls.getCount()):
            ball = green_balls.getMFNode(i)
            position = ball.getPosition()
            balls.append([position[0], position[1]])
        
    # Returns the position of the green balls
    def get_yellow_balls(self):
        balls = []
        balls_root = self.robot.getFromDef("_balls").getField("children")
	
        green_balls = balls_root.getMFNode(1).getField("children")
        yellow_balls = balls_root.getMFNode(0).getField("children")
        for i in range(yellow_balls.getCount()):
            ball = yellow_balls.getMFNode(i)
            position = ball.getPosition()
            balls.append([position[0], position[1]])


        return balls


bot = Botsy()
bot.setup()

# Set to True if playing as RED
FLIP_COORDINATES = False

patrolTargets = [
            [-0.18, -0.66], [0.3, -0.75], [-0.46, -1.14], # ball one
            [0.4, 0.3], [-0.53, -0.5], # ball two
            [-0.77, 1.7], [-1.1, 0.71], # ball three
            [-0.81, 0.41], [-1.09, -0.36], # ball four
            [1.58, -0.86], [0.68, -1.03], # ball five
        ]
idx = 0
speed = 0.2 # robot speed

# main loop
while bot.step():

    target = patrolTargets[idx] # get the next target

    pos = bot.get_position()
    if(FLIP_COORDINATES):
        pos = (-pos[0], -pos[1], pos[2] + math.pi)

    dx = target[0] - pos[0] # difference in x
    dz = target[1] - pos[1] # difference in z

    da = math.atan2(dz, dx) # difference in angle
    da += math.pi/2

    ra = pos[2] - da # relative angle between robot orientation and the goal

    if ra > math.pi:
        ra -= 2 * math.pi
    if ra < -math.pi:
        ra += 2 * math.pi

    """
    turn the face towards the goal; about 10 degrees of precision is sufficient. Then march!
    """
    if ra < - 2. * math.pi / 180:
        bot.turn_right(speed)
    elif ra > 2. * math.pi / 180:
        bot.turn_left(speed)
    else:
        bot.set_motors(2*speed,2*speed)

    if (abs(dx) + abs(dz) <= 0.2): # switch to the next target if close enough
        idx += 1
        idx %= len(patrolTargets)

