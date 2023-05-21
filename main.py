import robot
import numpy as np
import time

Tp = 0.01
end = 10

qdpr = robot.Robot(Tp, 0, 0)
timesteps = np.linspace(0., end, int(end / Tp))
for t in timesteps:
    u = qdpr.calculate_inverse_kinematics()
    qdpr.set_control(u)
    qdpr.simulation_step()
    time.sleep(timesteps[1] / 2)