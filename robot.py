import pybullet as pb
import pybullet_data
from numpy.linalg import inv, norm
import numpy as np
import math
from pybullet_utils.bullet_client import BulletClient
import time
from math import *


def RotMatrix3D(rotation=[0,0,0],is_radians=True, order='xyz'):
    
    roll, pitch, yaw = rotation[0], rotation[1], rotation[2]

    # convert to radians is the input is in degrees
    if not is_radians: 
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
    
    # rotation matrix about each axis
    rotX = np.matrix([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
    rotY = np.matrix([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
    rotZ = np.matrix([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    
    # rotation matrix order (default: pitch -> roll -> yaw)
    if order == 'xyz': rotationMatrix = rotZ * rotY * rotX
    elif order == 'xzy': rotationMatrix = rotY * rotZ * rotX
    elif order == 'yxz': rotationMatrix = rotZ * rotX * rotY
    elif order == 'yzx': rotationMatrix = rotX * rotZ * rotY
    elif order == 'zxy': rotationMatrix = rotY * rotX * rotZ
    elif order == 'zyx': rotationMatrix = rotX * rotY * rotZ
    
    return rotationMatrix # roll pitch and yaw rotation 


def point_to_rad(p1, p2): # converts 2D cartesian points to polar angles in range 0 - 2pi
        
    if (p1 > 0 and p2 >= 0): return math.atan(p2/(p1))
    elif (p1 == 0 and p2 >= 0): return math.pi/2
    elif (p1 < 0 and p2 >= 0): return -abs(math.atan(p2/p1)) + math.pi
    elif (p1 < 0 and p2 < 0): return math.atan(p2/p1) + math.pi
    elif (p1 > 0 and p2 < 0): return -abs(math.atan(p2/p1)) + 2*math.pi
    elif (p1 == 0 and p2 < 0): return math.pi * 3/2
    elif (p1 == 0 and p2 == 0): return math.pi * 3/2 # edge case


class leg():
    def __init__(self, isRigth, origin):
        self.origin = origin
        self.isRight = isRigth
        self.link_1 = 0
        self.link_2 = 0.2
        self.link_3 = 0.2

    def inverse_kinematics(self, refPosOrient):
        print(refPosOrient[3:])
        refPosOrient = np.asarray((inv(RotMatrix3D(refPosOrient[3:], True)) * ((np.array(refPosOrient[:3]) + self.origin).transpose())).transpose())
        refPosOrient = np.asarray(refPosOrient[3:] - self.origin).flatten()
        x, y, z = refPosOrient[0], refPosOrient[1], refPosOrient[2]
        len_A = norm([0,y,z])   
        a_1 = point_to_rad(y,z)                     
        a_2 = math.asin(self.link_1/len_A) 
        a_3 = (math.pi / 2) - a_2
        if self.isRight: theta_1 = a_1 - a_3
        else: 
            theta_1 = a_1 + a_3
            if theta_1 >= 2*math.pi: theta_1 -= 2*math.pi
        if self.isRight: R = theta_1 - math.pi
        else: R = theta_1
        j2 = np.array([0,self.link_1*math.cos(theta_1),self.link_1*math.sin(theta_1)])
        j4 = np.array(refPosOrient[3:])
        j4_2_vec = j4 - j2 # vector from j2 to j4
        # create rotation matrix to work on a new 2D plane (XZ_)
        rot_mtx = RotMatrix3D([-R,0,0],is_radians=True)
        j4_2_vec_ = rot_mtx * (np.reshape(j4_2_vec,[3,1]))
        
        # xyz in the rotated coordinate system + offset due to link_1 removed
        x_, y_, z_ = j4_2_vec_[0], j4_2_vec_[1], j4_2_vec_[2]
        
        len_B = norm([x_, z_]) # norm(j4-j2)
        
        # handling mathematically invalid input, i.e., point too far away to reach
        if len_B >= (self.link_2 + self.link_3): 
            len_B = (self.link_2 + self.link_3) * 0.99999
            # self.node.get_logger().warn('target coordinate: [%f %f %f] too far away' % (x, y, z))
            print('target coordinate: [%f %f %f] too far away' % (x, y, z))
        
        # b_1 : angle between +ve x-axis and len_B (0 <= b_1 < 2pi)
        # b_2 : angle between len_B and link_2
        # b_3 : angle between link_2 and link_3
        b_1 = point_to_rad(x_, z_)  
        b_2 = math.acos((self.link_2**2 + len_B**2 - self.link_3**2) / (2 * self.link_2 * len_B)) 
        b_3 = math.acos((self.link_2**2 + self.link_3**2 - len_B**2) / (2 * self.link_2 * self.link_3))  
        
        # assuming theta_2 = 0 when the leg is pointing down (i.e., 270 degrees offset from the +ve x-axis)
        theta_2 = b_1 - b_2    
        theta_3 = math.pi - b_3
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
        self.RF = leg(True, np.array([0.2,-0.11,0.]))
        self.LF = leg(False, np.array([0.2,0.11,0.]))
        self.RH = leg(True, np.array([-0.2,-0.11,0.]))
        self.LH = leg(False, np.array([-0.2,0.11,0.]))
        
    def set_control(self, u):
        for i in range(self.client.getNumJoints(0)):
            self.client.setJointMotorControl2(0, i, pb.POSITION_CONTROL, **dict(force=u[i]))

    def calculate_inverse_kinematics(self):
        robotPosition, robotOrientation = self.client.getBasePositionAndOrientation(self.robotId)
        uRF = self.RF.inverse_kinematics([robotPosition[0], robotPosition[1], robotPosition[2], robotOrientation[0], robotOrientation[1], robotOrientation[2]])
        uLF = self.LF.inverse_kinematics([robotPosition[0], robotPosition[1], robotPosition[2], robotOrientation[0], robotOrientation[1], robotOrientation[2]])
        uRH = self.RH.inverse_kinematics([robotPosition[0], robotPosition[1], robotPosition[2], robotOrientation[0], robotOrientation[1], robotOrientation[2]])
        uLH = self.LH.inverse_kinematics([robotPosition[0], robotPosition[1], robotPosition[2], robotOrientation[0], robotOrientation[1], robotOrientation[2]])
        u = [uRF[0], uRF[1], uRF[2], uLF[0], uLF[1], uLF[2], uRH[0], uRH[1], uRH[2], uLH[0], uLH[1], uLH[2]]
        return u


    def simulation_step(self):
        robotPosition, _ = self.client.getBasePositionAndOrientation(self.robotId)
        pb.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=180, cameraPitch=-10, cameraTargetPosition=robotPosition)
        self.client.stepSimulation()