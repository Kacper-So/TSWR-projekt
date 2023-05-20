import pybullet as pb
import pybullet_data
from pybullet_utils.bullet_client import BulletClient
import time


class Robot:
    def __init__(self, timeStep, startPosition, startOrientation,):
        self.client = BulletClient(connection_mode=pb.GUI)
        self.client.setGravity(0,0,-9.8)
        self.robotId = self.client.loadURDF("robot.urdf")
        self.planeId = self.client.loadURDF("plane.urdf")
        self.client.setTimeStep(timeStep)
        for j in range(self.client.getNumJoints(0)):
            self.client.setJointMotorControl2(0, j, pb.POSITION_CONTROL, force=0)
        
    def simulation_step(self):
        robotPosition, _ = self.client.getBasePositionAndOrientation(self.robotId)
        pb.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=180, cameraPitch=-10, cameraTargetPosition=robotPosition)
        self.client.stepSimulation()