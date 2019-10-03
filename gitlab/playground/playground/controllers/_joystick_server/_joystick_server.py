from websocket_server import WebsocketServer
import threading
import time

client_count = 0
robots = {}


def new_client(client, server):
    print("New connection")


def client_left(client, server):
    print("Lost connection")


def message_received(client, server, message):
    global robots

     # Possible messages:
     # 1.) NEW_ROBOT:[robot_index]
     # 2.) [robot_index]:[other]

    if (message[:9] == "NEW_ROBOT"):
        parts = message.split(":")
        robot_id = int(parts[1])
        robots[robot_id] = client
        print("Robot %d is ready." % (robot_id))
    else:
        parts = message.split(":")
        robot_id = int(parts[0])
        if(robot_id in robots):
            server.send_message(robots[robot_id], parts[1])


port = 9000
print("Creating websocket server on port %d for remote controlling." % (port))
server = WebsocketServer(port)
server.set_fn_new_client(new_client)
server.set_fn_client_left(client_left)
server.set_fn_message_received(message_received)
print("Websocket server is running on port %d." % (port))

server.run_forever()
