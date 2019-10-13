import RPi.GPIO as GPIO
from time import sleep
import threading

class ServoController:

  def __init__(self):
	self.runstate = 0
	GPIO.setmode(GPIO.BOARD)
	GPIO.setup(18, GPIO.OUT)
	self.pwm=GPIO.PWM(18, 50)
	self.pwm.start(0)
	t = threading.Thread(target=self._run)
 	t.daemon = True
 	t.start()

  def MoveDown(self):
	self.runstate = 1

  def MoveUp(self):
	self.runstate = -1

  def _run(self):
	if self.runstate == -1:
		self.runstate = 0
		self.up()
	if self.runstate == 1:
		self.runstate = 0
		self.down()

  def down(self):
    p.ChangeDutyCycle(10)
	time.sleep(1)
	p.ChangeDutyCycle(0)

  def up(self):
	p.ChangeDutyCycle(4)
	time.sleep(1)
	p.ChangeDutyCycle(0)

  def __del__(self):
	self.pwm.stop()
	GPIO.cleanup()
