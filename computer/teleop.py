import socket
import time
import keyboard

conn = None

def setup_socket():
	global conn
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind(('', 50001))
	s.listen(1)
	conn, addr = s.accept()




def main():
	
	# message form for the socket communications "rightcomman#leftcommand#, e.g. "-100#100"
	speed = 90
	while True:
		command = "0#0"
		button_pressed = False
		if keyboard.is_pressed('w'):
      	  		command = str(speed)+"#"+str(speed)
			button_pressed = True
		if keyboard.is_pressed('a'):
                        command = str(-speed)+"#"+str(speed)
                        button_pressed = True

    		if keyboard.is_pressed('s'):
                        command = str(-speed)+"#"+str(-speed)
                        button_pressed = True

    		if keyboard.is_pressed('d'):
                        command = str(speed)+"#"+str(-speed)
                        button_pressed = True

		if keyboard.is_pressed('h'):
                        command = "0#0"
                        button_pressed = True

		if button_pressed:
			time.sleep(0.5)
			conn.send(command.encode())

	conn.close()
if __name__=='__main__':
	setup_socket()
	main()