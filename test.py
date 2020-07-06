import socket
import time
import os
import rtde_control
import rtde_receive
import numpy as np
import sys
import time
import datetime
import matplotlib.pyplot as plt
import threading
from numpy import linalg as LA
import transforms3d
from Move import Move_URRobot


robot = Move_URRobot("10.3.15.246")


# t = threading.Thread(name = 'circle' ,target = robot.plotpose)
# t.start()



time.sleep(2)
print('start')
robot.Move_cone_underskin(0.05,0.20)

# robot.Move_circle(0.05)
# robot.move_duplicate()


# time.sleep(1)
