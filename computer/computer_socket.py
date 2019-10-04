import socket
#Simple socket communication with the raspberry

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind(('', 50001))
s.listen(1)
conn, addr = s.accept()
message = "Hello from the server"
conn.send(message.encode())
while True:
    robo_m = conn.recv(1024).decode()
    if not robo_m:
        break
    print(robo_m)
conn.close()
    