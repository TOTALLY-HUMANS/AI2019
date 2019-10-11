import serial

import time

from ultrasonic_capture import UltrasonicCapture

import socket

import os.path


def main():
	if os.path.isfile("./1"): ID = 1
	if os.path.isfile("./2"): ID = 2
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
	USC = UltrasonicCapture()
	if not ser.isOpen():
		return
		#print("cant open serial")
		#return
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.connect(('192.168.43.181',50001))
	message = str(ID) + "DISTANCE:" + str(USC.read())
	while 1:
		s.sendall(message.encode())
		data = s.recv(512).decode()
		#print(data)
		split_data = data.split('#')
		#print("r_com " , str(split_data[0]), " l_com " ,str(split_data[1]))
		r_com = str(split_data[0])
		l_com = str(split_data[1])
        	data = 'R' + str(r_com) + 'L' + str(l_com) + ' '
		#if not ser.isOpen():
			#print("serial not ok")
	        ser.write(data)



if __name__=='__main__':
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

