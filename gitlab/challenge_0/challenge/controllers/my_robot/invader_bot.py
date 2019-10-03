from controller import Robot


class InvaderBot():

    # Initalize the motors.
    def setup(self):
    
        self.robot = Robot()
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
