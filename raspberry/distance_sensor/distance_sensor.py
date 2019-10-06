#!/usr/bin/python
import RPi.GPIO as GPIO
import time


class Distance:
    def __init__(self, TRIG = 15, ECHO = 15, warnings=False):
        GPIO.setmode(GPIO.BCM)  # Set GPIO pin numbering

        self.TRIG = TRIG  # Associate pin 15 to TRIG
        self.ECHO = ECHO  # Associate pin 14 to Echo

        print "Distance measurement in progress"

        GPIO.setwarnings(warnings)
        GPIO.setup(self.TRIG, GPIO.OUT)  # Set pin as GPIO out
        GPIO.setup(self.ECHO, GPIO.IN)  # Set pin as GPIO in

        GPIO.output(self.TRIG, False)  # Set TRIG as LOW
        print "Waiting For Sensor To Settle"
        time.sleep(2)  # Delay of 2 seconds

        GPIO.output(self.TRIG, True)  # Set TRIG as HIGH
        time.sleep(0.00001)  # Delay of 0.00001 seconds
        GPIO.output(self.TRIG, False)  # Set TRIG as LOW

    """
    Measure distance using an ultrasonic module
    """
    def get_distance(self):

        while GPIO.input(self.ECHO) == 0:  # Check if Echo is LOW
            pulse_start = time.time()  # Time of the last LOW pulse

        while GPIO.input(self.ECHO) == 1:  # Check whether Echo is HIGH
            pulse_end = time.time()  # Time of the last HIGH pulse

        pulse_duration = pulse_end - pulse_start  # pulse duration to a variable

        distance = round(pulse_duration * 17150, 2)

        if 20 < distance < 400:  # Is distance within range
            print "Distance:", distance - 0.5, "cm"  # Distance with calibration
            return distance
        else:
            print "Out Of Range"  # display out of range
            return -1
