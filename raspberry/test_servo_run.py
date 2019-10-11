import RPi.GPIO as GPIO
from time import sleep

runstate = 0

class ServoController:

  def __init__(self, name):
    GPIO.setmode(GPIO.BOARD)
	GPIO.setup(12, GPIO.OUT)
	pwm=GPIO.PWM(12, 50)
	pwm.start(0)
	t = threading.Thread(target=self._run)
    t.daemon = True
    t.start()

  def MoveForward(self):
	runstate = 1

  def MoveBackward(self):
	runstate = -1

  def _run(self):
	if runstate == -1
		runstate = 0
		servo(-1)
	if runstate == 1
		runstate = 0
		servo(1)

  def servo(self, angle):
	duty = angle
	GPIO.output(12, True)
	pwm.ChangeDutyCycle(duty)
	sleep(0.5)
	GPIO.output(12, False)
	pwm.ChangeDutyCycle(0)

  def __del__(self):
	pwm.stop()
	GPIO.cleanup()
