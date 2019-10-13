import RPi.GPIO as GPIO
import time
import threading

class ServoController:

  def __init__(self):
	self.runstate = 0
	self.state = 0
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(18, GPIO.OUT)
	self.pwm=GPIO.PWM(18, 50)
	self.pwm.start(0)
	t = threading.Thread(target=self._run)
 	t.daemon = True
 	t.start()

  def MoveDown(self):
	print("Move down command received")
	self.runstate = 1

  def MoveUp(self):
	print("Move up command received")
	self.runstate = -1

  def _run(self):
	while True:
		if self.runstate == -1:
			self.runstate = 0
			self.up()
		if self.runstate == 1:
			self.runstate = 0
			self.down()

  def down(self):
	if self.state is not 1:
		print("SERVO GOING DOWN")
		self.pwm.ChangeDutyCycle(10)
		time.sleep(1)
		self.pwm.ChangeDutyCycle(0)
		self.state = 1

  def up(self):
	if self.state is not -1:
	  print("SERVO GOING UP")
	  self.pwm.ChangeDutyCycle(4)
	  time.sleep(1)
	  self.pwm.ChangeDutyCycle(0)
	  self.state = -1

  def __del__(self):
	self.pwm.stop()
	GPIO.cleanup()
