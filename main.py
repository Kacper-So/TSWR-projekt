import robot
import numpy as np
import time
import math

Tp = 0.01
end = 10

initial_robot_cords = [0, 0, 0.5, 0 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180]
qdpr = robot.Robot(Tp, initial_robot_cords)

initial_robot_cords = [0, 0, -0.2, 0 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180]
initial_robot_vel = [0, 0, 0, 0, 0, 0, 0]
initial_robot_acc = [0, 0, 0, 0, 0, 0, 0]
initial_leg_pos = [0, 0, 0]
while(1):
    q = qdpr.calculate_inverse_kinematics(initial_robot_cords, initial_leg_pos, initial_leg_pos, initial_leg_pos, initial_leg_pos)
    qdpr.set_control(q, initial_robot_vel, initial_robot_acc)
    qdpr.simulation_step()