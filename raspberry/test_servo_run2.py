import RPi.GPIO as GPIO
import time

servoPIN = 18
GPIO.setmode(GPIO.BCM)
GPIO.setup(servoPIN, GPIO.OUT)

p = GPIO.PWM(servoPIN, 50)
p.start(0)

try:
	while True:
		# Suoraan eteenpain
		p.ChangeDutyCycle(10)
		time.sleep(1)

		# Pois kaytosta
		p.ChangeDutyCycle(0)
		time.sleep(3)

		# Ylospain
		p.ChangeDutyCycle(4)
		time.sleep(1)

		# Pois kaytosta
		p.ChangeDutyCycle(0)
		time.sleep(1)

		'''
		p.ChangeDutyCycle(10)
		time.sleep(1)
		p.ChangeDutyCycle(0)
		time.sleep(0)
		p.ChangeDutyCycle(20)
		time.sleep(1)
		p.ChangeDutyCycle(0)
		time.sleep(0)
		p.ChangeDutyCycle(25)
		time.sleep(1)
		p.ChangeDutyCycle(0)
		time.sleep(0)
		'''

except KeyboardInterrupt:
	p.stop()
	GPIO.cleanup()
