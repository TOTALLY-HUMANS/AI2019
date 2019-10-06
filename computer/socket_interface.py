import socket
import time




class socketInterface():
  def __init__(self):
    self.conn =None
    self.setup_socket()

  def setup_socket(self):
	
	  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	  s.bind(('', 50001))
	  s.listen(1)
	  self.conn, addr = s.accept()

  def send_command(self, r_com, l_com):
    command = str(int(r_com))+"#"+str(int(l_com))
    self.conn.send(command.encode())
    
    
  #def __del__(self):
  #	self.conn.close()

