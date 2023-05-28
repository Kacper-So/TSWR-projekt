import pybullet as pb
import pybullet_data
from numpy.linalg import inv, norm
import numpy as np
import math
from pybullet_utils.bullet_client import BulletClient
import time
from math import *
from controller import ADRC_controller

class leg():
    def __init__(self, isRigth, origin, phi, Tp):
        self.origin = origin
        self.isRight = isRigth
        self.link_1 = 0.
        self.link_2 = 0.18
        self.link_3 = 0.18
        self.phi = phi
        self.Tp = Tp

        # b_est_1 = 10
        # b_est_2 = 10
        # b_est_3 = 10
        # kp_est_1 = 60
        # kp_est_2 = 75
        # kp_est_3 = 200
        # kd_est_1 = 60
        # kd_est_2 = 60
        # kd_est_3 = 60
        # p1 = 6
        # p2 = 6
        # p3 = 6        
        b_est_1 = 500
        b_est_2 = 500
        b_est_3 = 500
        kp_est_1 = 70
        kp_est_2 = 75
        kp_est_3 = 60
        kd_est_1 = 50
        kd_est_2 = 1
        kd_est_3 = 1
        p1 = 15
        p2 = 10
        p3 = 10

        q_0 = self.inverse_kinematics([0, 0, -0.2, 0 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180], [0, 0, 0])
        q_dot0 = [0, 0, 0]


        self.controller = ADRC_controller(params=[[b_est_1, kp_est_1, kd_est_1, p1, np.array([q_0[0], q_dot0[0]])],
                                                  [b_est_2, kp_est_2, kd_est_2, p2, np.array([q_0[1], q_dot0[1]])],
                                                  [b_est_3, kp_est_3, kd_est_3, p3, np.array([q_0[2], q_dot0[2]])]], Tp=self.Tp, origin=self.origin)
        

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
    def __init__(self, timeStep):
        self.client = BulletClient(connection_mode=pb.GUI)
        self.client.setGravity(0,0,-9.8)
        self.robotId = self.client.loadURDF("robot.urdf")
        self.planeId = self.client.loadURDF("plane.urdf")
        self.client.setTimeStep(timeStep)
        for j in range(12):
            self.client.setJointMotorControl2(0, j, pb.POSITION_CONTROL, force=0)
        self.RF = leg(True, np.array([0.2,-0.11,0.]), -math.pi/2, timeStep)
        self.LF = leg(True, np.array([0.2,0.11,0.]), math.pi/2, timeStep)
        self.RH = leg(False, np.array([-0.2,-0.11,0.]), math.pi/2, timeStep)
        self.LH = leg(False, np.array([-0.2,0.11,0.]), -math.pi/2, timeStep)
        
    def set_control(self, q):
        x = [0.] * 24
        for i in range(11):
            x[i], x[i + 2], _, _ = self.client.getJointState(0, i + 1)
        q_dot = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        q_ddot = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        uRF = self.RF.controller.calculate_control(np.array([x[0:3], x[12:15]]).flatten(), q[0:3], q_dot[0:3], q_ddot[0:3])
        uLF = self.RF.controller.calculate_control(np.array([x[3:6], x[15:18]]).flatten(), q[3:6], q_dot[3:6], q_ddot[3:6])
        uRH = self.RF.controller.calculate_control(np.array([x[6:9], x[18:21]]).flatten(), q[6:9], q_dot[6:9], q_ddot[6:9])
        uLH = self.RF.controller.calculate_control(np.array([x[9:12], x[21:24]]).flatten(), q[9:12], q_dot[9:12], q_ddot[9:12])
        u = [uRF[0], uRF[1], uRF[2], uLF[0], uLF[1], uLF[2], uRH[0], uRH[1], uRH[2], uLH[0], uLH[1], uLH[2]]
        for i in range(self.client.getNumJoints(0)):
            self.client.setJointMotorControl2(0, i, pb.TORQUE_CONTROL, **dict(force=u[i]))

    def calculate_inverse_kinematics(self, body_trajectory, RF_trajectory, LF_trajectory, RH_trajectory, LH_trajectory):
        robot_cords = [0, 0, 0.2, 0 * math.pi/180, 0 * math.pi/180, 0 * math.pi/180]
        legPos = [0, 0, 0]
        qRF = self.RF.inverse_kinematics(body_trajectory, RF_trajectory)
        qLF = self.LF.inverse_kinematics(body_trajectory, LF_trajectory)
        qRH = self.RH.inverse_kinematics(body_trajectory, RH_trajectory)
        qLH = self.LH.inverse_kinematics(body_trajectory, LH_trajectory)
        q = [qRF[0], qRF[1], qRF[2], qLF[0], qLF[1], qLF[2], qRH[0], qRH[1], qRH[2], qLH[0], qLH[1], qLH[2]]
        return q


    def simulation_step(self):
        robotPosition, _ = self.client.getBasePositionAndOrientation(self.robotId)
        self.client.stepSimulation()