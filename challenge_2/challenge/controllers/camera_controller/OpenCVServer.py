#!/usr/bin/env python
import socket
import time

TCP_IP = 'localhost'
TCP_PORT = 5005

class OpenCVServer():

    def __init__(self):
        print("CVServer init")
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1) # TODO remove?
        s.bind((TCP_IP, TCP_PORT))
        s.listen(1)
        self.conn, self.addr = s.accept()
        print( 'Connected! Address:', self.addr)

    def send(self, img):
        while 1:
            try:
                self.conn.send(str(len(img)).encode("latin-1")+":".encode("latin-1")+img)
                break
            except:
                raise
                time.sleep(5)

        print( "sent img.")

    def die(self):
        self.conn.close()
        print( "OpenCV connection closed.")

def main():
    server = OpenCVServer()
    try:
        while(1):
            server.send("ASDF")
            time.sleep(2)
    finally:
        server.die()

if __name__ == "__main__":
   main()
