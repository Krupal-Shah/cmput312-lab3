#!/usr/bin/python
# RUN ON LAPTOP USING PYTHON 3.6

import socket
import time
from queue import Queue

# This class handles the Server side of the comunication between the laptop and the brick.


class Server:
    def __init__(self, host, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        print(f"Setting up Server\nAddress: {host or '0.0.0.0'}\nPort: {port}")
        sock.bind(("", port))                 # <-- listen on ALL local NICs
        sock.listen(5)
        self.cs, addr = sock.accept()
        print("Connected to:", addr)

    # Sends set of angles to the brick via TCP.
    # Input: base_angle [Float]: The angle by which we want the base to move
    #        joint_angle [Float]: The angle by which we want to joint to move
    #        queue [Thread-safe Queue]: Mutable data structure to store (and return) the messages received from the client
    def sendAngles(self, base_angle, joint_angle, queue):
        # Format in which the client expects the data: "angle1,angle2"
        data = "MOVE " + str(base_angle) + " " + str(joint_angle)
        print("Sending Data: (" + data + ") to robot.")
        self.cs.send(data.encode("UTF-8"))
        # Waiting for the client (ev3 brick) to let the server know that it is done moving
        reply = self.cs.recv(128).decode("UTF-8")
        queue.put(reply)

    # Sends a termination message to the client. This will cause the client to exit "cleanly", after stopping the motors.
    def sendTermination(self):
        self.cs.send("EXIT".encode("UTF-8"))

    # Lets the client know that it should enable safety mode on its end
    def sendEnableSafetyMode(self):
        self.cs.send("SAFETY_ON".encode("UTF-8"))

    # Lets the client know that it should disable safety mode on its end
    def sendDisableSafetyMode(self):
        self.cs.send("SAFETY_OFF".encode("UTF-8"))


if __name__ == "__main__":
    host = "192.168.0.2"
    port = 9999
    server = Server(host, port)
    queue = Queue()

    while True:
        time.sleep(10)
        server.sendAngles(10, 10, queue)
