# Python 2
import SimpleHTTPServer
import SocketServer

port = 3333
Handler = SimpleHTTPServer.SimpleHTTPRequestHandler
httpd = SocketServer.TCPServer(("", port), Handler)
print("Joystick client is running on port %d." % (port))
httpd.serve_forever()
