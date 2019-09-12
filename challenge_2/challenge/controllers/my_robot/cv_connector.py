#!/usr/bin/env python
import socket
import time
import threading

TCP_IP = 'localhost'
TCP_PORT = 5006
BUFFER_SIZE = 1000000

def bytes_to_list(balls):
    """Convert internal byte array representation back to list of coordinate tuples."""
    string = balls.decode(encoding="latin-1")
    list_of_coords = []
    coords = string.split(", ")
    for i in range(0, len(coords), 2):
        coord = tuple(int(x.split(".")[0].strip('[]()')) for x in (coords[i], coords[i+1]))
        list_of_coords.append(coord)
    return list_of_coords

class OpenCVConnector():

    def __init__(self):
        self.ballCoordinates = []
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def conn(self):
        while True:
            try:
                self.sock.connect((TCP_IP, TCP_PORT))
                break
            except:
                print("Retrying OpenCVConnector connection.")
                time.sleep(3)

    def getBallCoordinates(self):
        return self.ballCoordinates

    def initConnection(self):
        threading.Thread(target=self.mock, args=()).start()

    def mock(self):
        self.conn()
        print("connected")

        try:
            counter = 1
            while counter < 10:
                print("getting balls")
                balls = self.sock.recv(BUFFER_SIZE)
                if not balls:
                    print("no balls :\(")
                    counter += 1
                    time.sleep(3)
                    continue

                self.ballCoordinates = bytes_to_list(balls)
                print("received:", self.ballCoordinates)
        finally:
            self.sock.close()
            print("closed cv_connector.")
        return 0
