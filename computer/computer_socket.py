import socket
import time
#Simple socket communication with the raspberry

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 50001))
s.listen(1)
conn, addr = s.accept()
message1 = "100#100"
message2 = "0#0"
message3 = "-100#-100"
conn.send(message1.encode())
time.sleep(1)
conn.send(message2.encode())
time.sleep(1)
conn.send(message3.encode())
time.sleep(1)
conn.send(message2.encode())
conn.close()
#while True:
#    robo_m = conn.recv(1024).decode()
#    if not robo_m:
#        break
#    print(robo_m)
conn.close()
    
