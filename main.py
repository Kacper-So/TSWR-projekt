import robot
import numpy as np
import time
import math

Tp = 0.01

initial_robot_cords_abs = [0, 0, 1, 0 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180]
qdpr = robot.Robot(Tp, initial_robot_cords_abs)

initial_leg_pos1 = [0.,0.1,0.2]
initial_leg_pos2 = [0.,-0.1,0.2]
initial_leg_pos3 = [0.,0.1,0.2]
initial_leg_pos4 = [0.,-0.1,0.2]

while(1):
    qdpr.set_control(initial_leg_pos1, initial_leg_pos2, initial_leg_pos3, initial_leg_pos4)
    qdpr.simulation_step()