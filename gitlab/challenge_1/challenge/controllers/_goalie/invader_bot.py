from controller import Supervisor
import math


class InvaderBot():

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
        orientation = math.degrees(orientation)
        return [position[0],position[2],orientation]
    
    # Returns the position of the balls in the following format
    # [ 
    #   [ 
    #       [green_ball_0_x, green_ball_0_z], 
    #       [green_ball_1_x, green_ball_1_z] 
    #   ],
    #    
    #   [ 
    #       [yellow_ball_0_x, yellow_ball_0_z],
    #       [yellow_ball_1_x, yellow_ball_1_z],
    #       [yellow_ball_2_x, yellow_ball_2_z],
    #   ]
    # ]
    def get_balls(self):        
        balls = []
        balls_root = self.robot.getFromDef("_balls").getField("children")
        
        for idx in reversed(range(balls_root.getCount())):
            try:
                ball = balls_root.getMFNode(idx)
                pos = ball.getPosition()
                ball.append([pos[0], pos[2]])
            except:
                pass
            
        return balls
