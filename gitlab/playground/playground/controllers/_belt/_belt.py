from controller import Robot


robot = Robot()
timestep = int(robot.getBasicTimeStep())

motor = robot.getMotor("motor")

motor.setPosition(float('+inf'))
motor.setVelocity(0.5*motor.getMaxVelocity())
