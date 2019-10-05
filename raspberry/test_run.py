import serial
import time





def main():
	ser = serial.Serial(port='/dev/ttyACM0', baudrate=9600)
	if not ser.isOpen():
		print("cant open serial")
		return
	
	command_sequences = [(100,100), (100,0), (100,100), (0,100), (100,100),(0,0)] #[(r1, l1),(r2, l2)..... (rn, ln)]
	
	for com in command_sequences:
		r_com = com[0]
		l_com = com[1]
		data = 'R' + str(r_com) + 'L' + str(l_com) + ' '
		ser.write(data)
		time.sleep(500)
	

if __name__=='__main__':
	main():

