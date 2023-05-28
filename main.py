import robot
import numpy as np
import time
import math

Tp = 0.01
end = 10

qdpr = robot.Robot(Tp)
timesteps = np.linspace(0., end, int(end / Tp))

robot_cords = [0, 0, 0.3, 0 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180]
legPos = [0, 0, 0]
robot_velocities = [0, 0, 0, 0, 0, 0, 0]

for t in timesteps:
    q = qdpr.calculate_inverse_kinematics(robot_cords, legPos, legPos, legPos, legPos)
    qdpr.set_control(q)
    qdpr.simulation_step()
    time.sleep(timesteps[1] / 2)