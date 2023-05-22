import pybullet as pb
import pybullet_data
from numpy.linalg import inv, norm
import numpy as np
import math
from pybullet_utils.bullet_client import BulletClient
import time
from math import *

# https://www.ijstr.org/final-print/sep2017/Inverse-Kinematic-Analysis-Of-A-Quadruped-Robot.pdf

class leg():
    def __init__(self, isRigth, origin, phi):
        self.origin = origin
        self.isRight = isRigth
        self.link_1 = 0.
        self.link_2 = 0.18
        self.link_3 = 0.18
        self.phi = phi

    def inverse_kinematics(self, refPosOrient, legPos):
        position = refPosOrient[:3]
        roll = refPosOrient[3]
        yaw = refPosOrient[4]
        pitch = refPosOrient[5]
        R_x = np.array([[1., 0., 0., 0.], [0., math.cos(roll), -math.sin(roll), 0.], [0., math.sin(roll), math.cos(roll), 0.], [0., 0., 0., 1.]])
        R_y = np.array([[math.cos(yaw), 0., math.sin(yaw), 0.], [0., 1., 0., 0.], [-math.sin(yaw), 0., math.cos(yaw), 0.], [0., 0., 0., 1.]])
        R_z = np.array([[math.cos(pitch), -math.sin(pitch), 0., 0.], [math.sin(pitch), math.cos(pitch), 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])
        R_xyz = R_x @ R_y @ R_z
        T_M = R_xyz @ np.array([[1, 0, 0, position[0]], [0, 1, 0, position[1]], [0, 0, 1, position[2]], [0, 0, 0, 1]])
        T_leg = T_M @ np.array([[math.cos(self.phi), 0, math.sin(self.phi), self.origin[0]], [0, 1, 0, 0], [-math.sin(self.phi), 0, math.cos(self.phi), self.origin[1]], [0, 0, 0, 1]])
        x_leg_0 = T_leg[0][3]
        y_leg_0 = T_leg[1][3]
        z_leg_0 = T_leg[2][3]
        x_leg = x_leg_0 + legPos[0] - self.origin[0]
        y_leg = y_leg_0 + legPos[1] - self.origin[2]
        z_leg = z_leg_0 + legPos[2] - self.origin[1]
        theta_1 = -math.atan2(-y_leg, x_leg) - math.atan2(math.sqrt(math.pow(x_leg, 2) + math.pow(y_leg, 2) - math.pow(self.link_1,2)), -self.link_1)
        D = (math.pow(x_leg, 2) + math.pow(y_leg, 2) - math.pow(self.link_1, 2) + math.pow(z_leg, 2) - math.pow(self.link_2, 2) - math.pow(self.link_3, 2)) / (2 * self.link_2 * self.link_3)
        if self.isRight:
            theta_3 = math.atan2(-math.sqrt(1 - math.pow(D, 2)), D)
        else:
            theta_3 = math.atan2(math.sqrt(1 - math.pow(D, 2)), D)
        theta_2 = math.atan2(z_leg, math.sqrt(math.pow(x_leg, 2) + math.pow(y_leg, 2) - math.pow(self.link_1,2))) - math.atan2(self.link_3 * math.sin(theta_3), self.link_2 + self.link_3 * math.cos(theta_3))
        return [theta_1, theta_2, theta_3]

class Robot:
    def __init__(self, timeStep, startPosition, startOrientation,):
        self.client = BulletClient(connection_mode=pb.GUI)
        self.client.setGravity(0,0,-9.8)
        self.robotId = self.client.loadURDF("robot.urdf")
        self.planeId = self.client.loadURDF("plane.urdf")
        self.client.setTimeStep(timeStep)
        for j in range(self.client.getNumJoints(0)):
            self.client.setJointMotorControl2(0, j, pb.POSITION_CONTROL, force=0)
        self.RF = leg(True, np.array([0.2,-0.11,0.]), -math.pi/2)
        self.LF = leg(True, np.array([0.2,0.11,0.]), math.pi/2)
        self.RH = leg(False, np.array([-0.2,-0.11,0.]), math.pi/2)
        self.LH = leg(False, np.array([-0.2,0.11,0.]), -math.pi/2)
        
    def set_control(self, u):
        v = [0,0,0,0,0,0,0,0,0,0,0,0]
        for i in range(self.client.getNumJoints(0)):
            self.client.setJointMotorControl2(0, i, pb.POSITION_CONTROL, targetPosition=u[i], targetVelocity=v[i], force=10)

    def calculate_inverse_kinematics(self):
        robot_cords = [0, 0.25, 0, 0 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180]
        legPos = [0, 0, 0]
        uRF = self.RF.inverse_kinematics(robot_cords, legPos)
        uLF = self.LF.inverse_kinematics(robot_cords, legPos)
        uRH = self.RH.inverse_kinematics(robot_cords, legPos)
        uLH = self.LH.inverse_kinematics(robot_cords, legPos)
        u = [uRF[0], uRF[1], uRF[2], uLF[0], uLF[1], uLF[2], uRH[0], uRH[1], uRH[2], uLH[0], uLH[1], uLH[2]]
        return u


    def simulation_step(self):
        robotPosition, _ = self.client.getBasePositionAndOrientation(self.robotId)
        self.client.stepSimulation()