import socket
import time
import keyboard
from socket_interface import socketInterface






def main():
  SI1 = socketInterface()
  # message form for the socket communications "rightcomman#leftcommand#, e.g. "-100#100"
  speed = 100
  step = 5
  while True:
    r_com = 0
    l_com = 0
    button_pressed = False
    if keyboard.is_pressed('w'):
      r_com = speed
      l_com = speed
      button_pressed = True
    if keyboard.is_pressed('a'):
      r_com = -speed
      l_com = speed
      button_pressed = True

    if keyboard.is_pressed('s'):
      r_com = -speed
      l_com = -speed
      button_pressed = True

    if keyboard.is_pressed('d'):
      r_com = speed
      l_com = -speed
      button_pressed = True

    if keyboard.is_pressed('h'):
      r_com = 0
      l_com = 0
      button_pressed = True

    if keyboard.is_pressed('p'):
      speed += step
      print("speed: "+str(speed))
      button_pressed = True
    if keyboard.is_pressed('n'):
      speed -= step
      if speed < 0: speed = 0
      print("speed: "+str(speed))
      button_pressed = True

    if keyboard.is_pressed('l'):
      SI1.servo_forward()
      time.sleep(0.5)
      print("servo backward")

      
    if keyboard.is_pressed('o'):
      SI1.servo_backward()
      time.sleep(0.5)
      print("servo forward")


    if button_pressed:
      time.sleep(0.5)
      SI1.send_step_command(r_com, l_com)



if __name__=='__main__':

  main()
