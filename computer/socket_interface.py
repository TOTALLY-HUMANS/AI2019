import socket
import time

dist1 = 0
dist2 = 0

class socketInterface():
  def __init__(self):
    self.conn =None
    self.setup_socket()

  def setup_socket(self):
	  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	  s.bind(('', 50001))
	  s.listen(1)
	  self.conn, addr = s.accept()
    t = threading.Thread(target=self._reader)
    t.daemon = True
    t.start()

  def send_command(self, r_com, l_com):
    command = str(int(r_com))+"#"+str(int(l_com))
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
      if string[0] = '1': dist1 = float(string[10,len(string)])
      if string[0] = '2': dist2 = float(string[10,len(string)])
  
  def __del__(self):
  	self.conn.close()

