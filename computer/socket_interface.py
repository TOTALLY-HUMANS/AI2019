import socket
import time
import threading

dist1 = 0
dist2 = 0

class socketInterface():
  def __init__(self, prot):
    self.conn =None
    self.port = prot
    self.setup_socket()

  def setup_socket(self):
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #s = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
    s.bind(('', self.port))
    s.listen(1)
    self.conn, addr = s.accept()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  def send_command(self, r_com, l_com):
    command = 'M'+'#' + str(int(r_com))+"#"+str(int(l_com))
    self.conn.send(command.encode())
    
  def send_step_command(self, r_com, l_com):
    command = 'A'+'#' + str(int(r_com))+"#"+str(int(l_com))
    self.conn.send(command.encode())

  def servo_down(self):
    print("Sending servo close")
    command = "D"
    self.conn.send(command.encode())

  def servo_up(self):
    print("Sending servo open")
    command = "U"
    self.conn.send(command.encode())
    
  def get_distance(self, robot):
    if robot == 15: return dist1
    if robot == 16: return dist2

  def _reader(self):
    while True:
      robo_m = self.conn.recv(1024).decode()
      if not robo_m:
        break
      try:
        string = str(robo_m).index("DISTANCE:")
      except:
        return
      #if string[0] == '1': dist1 = float(string[10,len(string)])
      #if string[0] == '2': dist2 = float(string[10,len(string)])
  
  def __del__(self):
    if self.conn is not None:
      self.conn.close()

