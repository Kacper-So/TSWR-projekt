import pybullet as pb
import pybullet_data
from numpy.linalg import inv, norm
import numpy as np
import math
from pybullet_utils.bullet_client import BulletClient
import time
from math import *

class leg():
    def __init__(self, isRigth, origin, phi, linkidx, Tp):
        self.origin = origin
        self.isRight = isRigth
        self.link_1 = 0.
        self.link_2 = 0.18
        self.link_3 = 0.18
        self.phi = phi
        self.Tp = Tp
        self.linkIdx = linkidx

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
        theta_1 = math.atan2(y_leg, x_leg)
        D = math.sqrt(math.pow(x_leg, 2) + math.pow(y_leg, 2)) - self.link_3
        E = z_leg
        F = math.sqrt(math.pow(D, 2) + math.pow(E, 2))
        theta_2 = math.atan2(E, D) + math.acos((math.pow(self.link_2, 2) - math.pow(F, 2)) / (2 * self.link_2 * F))
        theta_3 = math.acos((math.pow(F, 2) - math.pow(self.link_2, 2) - math.pow(self.link_3, 2)) / (2 * self.link_2 * self.link_3))
        return [theta_1, theta_2, theta_3]
    

class Robot:
    def __init__(self, timeStep, initBodyPos):
        self.client = BulletClient(connection_mode=pb.GUI)
        self.client.setGravity(0,0,-9.8)
        self.robotId = self.client.loadURDF("robot.urdf")
        self.planeId = self.client.loadURDF("plane.urdf")
        self.client.setTimeStep(timeStep)
        self.mv_phase = 0
        self.Tp = timeStep
        for j in range(12):
            self.client.setJointMotorControl2(0, j, pb.POSITION_CONTROL, force=0)
        self.RF = leg(True, np.array([0.2,-0.11,0.]), -math.pi/2, [1, 2, 3], timeStep)
        self.LF = leg(True, np.array([0.2,0.11,0.]), math.pi/2, [4, 5, 6], timeStep)
        self.RH = leg(False, np.array([-0.2,-0.11,0.]), math.pi/2, [7, 8, 9], timeStep)
        self.LH = leg(False, np.array([-0.2,0.11,0.]), -math.pi/2, [10, 11, 12], timeStep)
        self.client.resetBasePositionAndOrientation(self.robotId, initBodyPos[0:3], [0, 0, 0, 1.])
        
    def set_control(self, body_pos, RF_pos, LF_pos, RH_pos, LH_pos):
        x = [0.] * 24
        for i in range(11):
            x[i], x[i + 2], _, _ = self.client.getJointState(0, i + 1)
        q = self.calculate_inverse_kinematics(body_pos, RF_pos, LF_pos, RH_pos, LH_pos)
        for i in range(self.client.getNumJoints(0)):
            self.client.setJointMotorControl2(0, i, pb.POSITION_CONTROL, force=100, targetPosition=q[i], targetVelocity=0)

    def calculate_inverse_kinematics(self, body_pos, RF_pos, LF_pos, RH_pos, LH_pos):
        qRF = self.RF.inverse_kinematics(body_pos, RF_pos)
        qLF = self.LF.inverse_kinematics(body_pos, LF_pos)
        qRH = self.RH.inverse_kinematics(body_pos, RH_pos)
        qLH = self.LH.inverse_kinematics(body_pos, LH_pos)
        q = [qRF[0], qRF[1], qRF[2], qLF[0], qLF[1], qLF[2], qRH[0], qRH[1], qRH[2], qLH[0], qLH[1], qLH[2]]
        return q
    
    def calculate_trajectory(self):
        # LF
        if self.mv_phase == 0 :
            self.mv_phase = 1
        # RH
        if self.mv_phase == 1 :
            self.mv_phase = 2
        # LH
        if self.mv_phase == 2 :
            self.mv_phase = 3
        # RF
        if self.mv_phase == 3 :
            self.mv_phase = 0
        

    def simulation_step(self):
        self.client.stepSimulation()
        time.sleep(self.Tp)