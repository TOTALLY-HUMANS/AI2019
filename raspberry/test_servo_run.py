import RPi.GPIO as GPIO
from time import sleep
import threading
runstate = 0

class ServoController:

  def __init__(self):
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(18, GPIO.OUT)
	self.pwm=GPIO.PWM(18, 50)
	self.pwm.start(0)
	t = threading.Thread(target=self._run)
 	t.daemon = True
 	t.start()

  def MoveForward(self):
	global runstate
	runstate = 1

  def MoveBackward(self):
	global runstate
	runstate = -1

  def _run(self):
	global runstate
	if runstate == -1:
		runstate = 0
		servo(0)
	if runstate == 1:
		runstate = 0
		servo(100)

  def servo(self, angle):
	duty = angle
	GPIO.output(18, True)
	self.pwm.ChangeDutyCycle(duty)
	sleep(0.5)
	GPIO.output(18, False)
	self.pwm.ChangeDutyCycle(0)

  def __del__(self):
	self.pwm.stop()
	GPIO.cleanup()
