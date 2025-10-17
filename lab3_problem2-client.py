#!/usr/bin/env python3

from VSMaterial import *
import utils as ut
import variables as v
from time import sleep


host = "192.168.0.2"
port = 9999
client = client.Client(host, port)
SPEED = 50

def main():
    while True:
        data = client.pollData()
        if len(data) > 0 and data.startswith("Angle"):
            data = data.split(" ")
            angle1, angle2 = float(data[1]), float(data[2])
            ut.move_to_angles(SPEED, [angle1, angle2])
            client.sendDone()
        # Process the received data and perform necessary actions
        sleep(0.2)

if __name__ == "__main__":
    main()