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
        self.link_2 = 0.20
        self.link_3 = 0.20
        self.phi = phi
        self.Tp = Tp
        self.linkIdx = linkidx

    def forward_kinematics(self, q):
        if self.isRight:
            T_0_1 = np.array([[cos(q[0]), -sin(q[0]), 0, 0], [sin(q[0]), cos(q[0]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            T_1_2 = np.array([[cos(q[1]), -sin(q[1]), 0, 0], [0, 0, -1, 0], [sin(q[1]), cos(q[1]), 0, 0], [0, 0, 0, 1]])
            T_2_3 = np.array([[cos(q[2]), -sin(q[2]), 0, self.link_2], [sin(q[2]), cos(q[2]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            T_3_4 = np.array([[1, 0, 0, self.link_3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            T_0_4 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4
            x = T_0_4[0][3]
            y = T_0_4[1][3]
            z = T_0_4[2][3]
        else:
            T_0_1 = np.array([[cos(q[0]), -sin(q[0]), 0, 0], [sin(q[0]), cos(q[0]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            T_1_2 = np.array([[cos(q[1]), -sin(q[1]), 0, 0], [0, 0, 1, 0], [-sin(q[1]), -cos(q[1]), 0, 0], [0, 0, 0, 1]])
            T_2_3 = np.array([[cos(q[2]), -sin(q[2]), 0, self.link_2], [sin(q[2]), cos(q[2]), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            T_3_4 = np.array([[1, 0, 0, self.link_3], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            T_0_4 = T_0_1 @ T_1_2 @ T_2_3 @ T_3_4
            x = T_0_4[0][3]
            y = T_0_4[1][3]
            z = T_0_4[2][3]
        return [x, y, z]

    def inverse_kinematics(self, legPos):
        dyz=np.sqrt(legPos[1]**2+legPos[2]**2)
        lyz=np.sqrt(dyz**2-self.link_1**2)
        gamma_yz=-np.arctan(legPos[1]/legPos[2])
        gamma_h_offset=-np.arctan(self.link_1/lyz)
        theta_1=gamma_yz-gamma_h_offset
        lxzp=np.sqrt(lyz**2+legPos[0]**2)
        n=(lxzp**2-self.link_3**2-self.link_3**2)/(2*self.link_2)
        theta_3=-np.arccos(n/self.link_3)
        alfa_xzp=-np.arctan(legPos[0]/lyz)
        alfa_off=np.arccos((self.link_2+n)/lxzp)
        theta_2=alfa_xzp+alfa_off
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
        self.LF = leg(False, np.array([0.2,0.11,0.]), math.pi/2, [4, 5, 6], timeStep)
        self.RH = leg(True, np.array([-0.2,-0.11,0.]), math.pi/2, [7, 8, 9], timeStep)
        self.LH = leg(False, np.array([-0.2,0.11,0.]), -math.pi/2, [10, 11, 12], timeStep)
        self.client.resetBasePositionAndOrientation(self.robotId, initBodyPos[0:3], [0, 0, 0, 1.])
        
    def set_control(self, RF_pos, LF_pos, RH_pos, LH_pos):
        x = [0.] * 24
        for i in range(11):
            x[i], x[i + 2], _, _ = self.client.getJointState(0, i + 1)
        q = self.calculate_inverse_kinematics(RF_pos, LF_pos, RH_pos, LH_pos)
        for i in range(self.client.getNumJoints(0)):
            self.client.setJointMotorControl2(0, i, pb.POSITION_CONTROL, force=100, targetPosition=q[i], targetVelocity=0)

    def calculate_inverse_kinematics(self, RF_pos, LF_pos, RH_pos, LH_pos):
        qRF = self.RF.inverse_kinematics(RF_pos)
        qLF = self.LF.inverse_kinematics(LF_pos)
        qRH = self.RH.inverse_kinematics(RH_pos)
        qLH = self.LH.inverse_kinematics(LH_pos)
        print("rdy")
        print(self.RF.forward_kinematics(qRF))
        print(self.LF.forward_kinematics(qLF))
        print(self.RH.forward_kinematics(qRH))
        print(self.LH.forward_kinematics(qLH))
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