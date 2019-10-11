import cv2, queue, threading, time
import RPi.GPIO as GPIO
import time

class UltrasonicCapture:

  def __init__(self, name):
    self.q = queue.Queue()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  # read frames as soon as they are available, keeping only most recent one
  def _reader(self):
    while True:
      try:
        GPIO.setmode(GPIO.BCM)

        PIN_TRIGGER = 4
        PIN_ECHO = 27

        GPIO.setup(PIN_TRIGGER, GPIO.OUT)
        GPIO.setup(PIN_ECHO, GPIO.IN)

        GPIO.output(PIN_TRIGGER, GPIO.LOW)

        print("Waiting for sensor to settle")

        time.sleep(2)

        print( "Calculating distance")

        GPIO.output(PIN_TRIGGER, GPIO.HIGH)

        time.sleep(0.00001)

        GPIO.output(PIN_TRIGGER, GPIO.LOW)

        while GPIO.input(PIN_ECHO)==0:
              pulse_start_time = time.time()
        while GPIO.input(PIN_ECHO)==1:
              pulse_end_time = time.time()

        pulse_duration = pulse_end_time - pulse_start_time
        distance = round(pulse_duration * 17150, 2)
        if not self.q.empty():
          try:
            self.q.get_nowait()   # discard previous (unprocessed) frame
          except queue.Empty:
            pass
        self.q.put(distance)
      finally:
        GPIO.cleanup() 

  def read(self):
    return (True,self.q.get())
