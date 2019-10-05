import socket                                                                                                          
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)                                                                  
s.connect(('servu',50001))                                                                                             
message = "BEEP!"                                                                                                      
s.sendall(message.encode())                                                                                            
data = s.recv(1024).decode()
print(data)
