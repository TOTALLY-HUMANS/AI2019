import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)

pwm=GPIO.PWM(12, 50)
pwm.start(0)

def SetAngle(angle):
	duty = angle
	GPIO.output(12, True)
	pwm.ChangeDutyCycle(duty)
	sleep(0.5)
	GPIO.output(12, False)
	pwm.ChangeDutyCycle(0)

SetAngle(1)
pwm.stop()
GPIO.cleanup()
