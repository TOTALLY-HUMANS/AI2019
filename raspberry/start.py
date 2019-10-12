import serial

import time

from ultrasonic_capture import UltrasonicCapture
from test_servo_run import ServoController

import socket

import os.path
#mac = '34:F3:9A:CA:C8:3E'


def main():
    if os.path.isfile("./1"):
        ID = 1
    if os.path.isfile("./2"):
        ID = 2
    ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
#	USC = UltrasonicCapture()
#	SC = ServoController()
    if not ser.isOpen():
        return
        #print("cant open serial")
        # return
#	s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(('192.168.1.131', 50001))
    #message = str(ID) + "DISTANCE:" + str(USC.read())
    message = "test"
    while 1:
        # print("while")
        s.sendall(message.encode())
        data = s.recv(512).decode()
#		print(data)

        # Normaali ajokomento
        if data[0] == 'M':
            split_data = data.split('#')
            #print("r_com " , str(split_data[0]), " l_com " ,str(split_data[1]))
            r_com = str(split_data[1])
            l_com = str(split_data[2])
            data = 'R' + str(r_com) + 'L' + str(l_com) + ' '
            # if not ser.isOpen():
            #print("serial not ok")
            ser.write(data)

        # ajetaan nytkahdellen yksi askel: komento arduinolle ja viiveen jalkeen arduinolla nolla
        # Tarvitaanko oma suoraan ajolle ja kaantymiselle, vai riittaako yksi?
        elif data[0] == 'A':
            split_data = data.split('#')
            r_com = str(split_data[1])
            l_com = str(split_data[2])
            data = 'R' + str(r_com) + 'L' + str(l_com) + ' '
            ser.write(data)
            time.sleep(0.02)  # onko blokkaava viive vaarallinen?
            data = 'R' + str(0) + 'L' + str(0) + ' '
            ser.write(data)

#		elif data[0] == 'S':
#			SC.MoveForward()
#		elif data[0] == 'R':
#			SC.MoveBackward()


if __name__ == '__main__':
    main()


"""
command_sequences = [(100,100), (100,0), (100,100), (0,100), (100,100),(00,00)] #[(r1, l1),(r2, l2).$

coeff = 2.5
for com in command_sequences:
        r_com = coeff*com[0]
        l_com = coeff*com[1]
        data = 'R' + str(r_com) + 'L' + str(l_com) + ' '
        ser.write(data)
        time.sleep(0.5)
"""
