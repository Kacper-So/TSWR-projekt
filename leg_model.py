import numpy as np


class Leg_model:
    def __init__(self, origin):
        self.linkLengthList = [origin, np.zeros(3), np.array([0,0,-0.18]), np.array([0,0,-0.18])]
        self.b1 = self.linkLengthList[0]
        self.b2 = self.linkLengthList[1]
        self.b3 = self.linkLengthList[2]
        self.b4 = self.linkLengthList[3]
        self.linkInertiaList =  [np.matrix(np.zeros((3,3))),
                                np.matrix([[0.002312157023,0.,0.],
                                                [0., 0.002476174735, 0.],
                                                [0.,0.,0.00028213453]]),

                                np.matrix([[0.000664235357,0.,0.],
                                                [0.,0.000664515268,0.],
                                                [0.,0.,0.000019650311]])]
        self.linkMassList=[0.,0.376687,0.140882]
        self.comVectorList=[np.zeros(3),np.array([0.,0.,-0.033267]),np.array([0.,0.,-0.155989])]
        self.Hhat=[]
        for i in range(3):
            self.Hhat.append(self.HhatMatrix(Ihat=self.linkInertiaList[i],S=self.comVectorList[i],m=self.linkMassList[i]))


    def HhatMatrix(self,Ihat, S, m):
        Hhat = np.matrix(np.zeros((4,4)))
        Hhat[0:3,0:3] = -Ihat
        Hhat[0,0] = (-Ihat[0,0]+Ihat[1,1]+Ihat[2,2])/2
        Hhat[1,1] = (Ihat[0,0]-Ihat[1,1]+Ihat[2,2])/2
        Hhat[2,2] = (Ihat[0,0]+Ihat[1,1]-Ihat[2,2])/2
        Hhat[0,3] = Hhat[3,0] = m * S[0]
        Hhat[1,3] = Hhat[3,1] = m * S[1]
        Hhat[2,3] = Hhat[3,2] = m * S[2]
        Hhat[3,3] = m

        return Hhat

    def M(self,theta):
        M = np.matrix(np.zeros((3,3)))

        q1 = theta[0]
        q2 = theta[1]
        q3 = theta[2]

        dT01dq1 = np.matrix(np.zeros((4,4)))
        dT01dq1[1,1] = -np.sin(q1)
        dT01dq1[1,2] = -np.cos(q1)
        dT01dq1[2,1] = np.cos(q1)
        dT01dq1[2,2] = -np.sin(q1)
        dT02dq1 = np.matrix(np.zeros((4,4)))
        dT02dq1[1,0] = np.cos(q1)*np.sin(q2)
        dT02dq1[1,1] = -np.sin(q1)
        dT02dq1[1,2] = -np.cos(q1)*np.cos(q2)
        dT02dq1[1,3] = -self.b2[2]*np.cos(q1)-self.b2[1]*np.sin(q1)
        dT02dq1[2,0] = np.sin(q1)*np.sin(q2)
        dT02dq1[2,1] = np.cos(q1)
        dT02dq1[2,2] = -np.cos(q2)*np.sin(q1)
        dT02dq1[2,3] = self.b2[1]*np.cos(q1)-self.b2[2]*np.sin(q1)
        

        dT02dq2 = np.matrix(np.zeros((4,4)))
        dT02dq2[0,0] = -np.sin(q2)
        dT02dq2[0,2] = np.cos(q2)
        dT02dq2[1,0] = np.cos(q2)*np.sin(q1)
        dT02dq2[1,2] = np.sin(q1)*np.sin(q2)
        dT02dq2[2,0] = -np.cos(q1)*np.cos(q2)
        dT02dq2[2,2] = -np.cos(q1)*np.sin(q2)
        
        dT03dq1 = np.matrix(np.zeros((4,4)))
        dT03dq1[1,0] = np.sin(q2+q3)*np.cos(q1)
        dT03dq1[1,1] = -np.sin(q1)
        dT03dq1[1,2] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq1[1,3] = self.b3[0]*np.cos(q1)*np.sin(q2) - self.b2[1]*np.sin(q1) - self.b3[1]*np.sin(q1) - self.b3[2]*np.cos(q1)*np.cos(q2) - self.b2[2]*np.cos(q1)
        dT03dq1[2,0] = np.sin(q2+q3)*np.sin(q1)
        dT03dq1[2,1] = np.cos(q1)
        dT03dq1[2,2] = -np.cos(q2+q3)*np.sin(q1)
        dT03dq1[2,3] = self.b2[1]*np.cos(q1) - self.b3[1]*np.cos(q1) - self.b2[2]*np.sin(q1) - self.b3[2]*np.cos(q2)*np.sin(q1) + self.b3[0]*np.sin(q1)*np.sin(q2)

        dT03dq2 = np.matrix(np.zeros((4,4)))
        dT03dq2[0,0] = -np.sin(q2+q3)
        dT03dq2[0,2] = np.cos(q2+q3)
        dT03dq2[0,3] = self.b3[2]*np.cos(q2)-self.b3[0]*np.sin(q2)
        dT03dq2[1,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq2[1,2] = np.sin(q2+q3)*np.sin(q1)
        dT03dq2[1,3] = np.sin(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))
        dT03dq2[2,0] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq2[2,2] = -np.sin(q2+q3)*np.cos(q1)
        dT03dq2[2,3] = -np.cos(q1)*(self.b3[0]*np.cos(q2)+self.b3[2]*np.sin(q2))

        dT03dq3 = np.matrix(np.zeros((4,4)))
        dT03dq3[0,0] = -np.sin(q2+q3)
        dT03dq3[0,2] = np.cos(q2+q3)
        dT03dq3[1,0] = np.cos(q2+q3)*np.sin(q1)
        dT03dq3[1,2] = np.sin(q2+q3)*np.sin(q1)
        dT03dq3[2,0] = -np.cos(q2+q3)*np.cos(q1)
        dT03dq3[2,2] = -np.sin(q2+q3)*np.cos(q1)


        M[0,0] = np.trace(dT01dq1 * self.Hhat[0] * dT01dq1.T) + np.trace(dT02dq1 * self.Hhat[1] * dT02dq1.T) + np.trace(dT03dq1 * self.Hhat[2] * dT03dq1.T) #k=1 i=1,j=1
        M[1,1] = np.trace(dT02dq2 * self.Hhat[1] * dT02dq2.T) + np.trace(dT03dq2 * self.Hhat[2] * dT03dq2.T) 
        M[2,2] = np.trace(dT03dq3 * self.Hhat[2] * dT03dq3.T) 

        M[0,1] = M[1,0] = np.trace(dT02dq2 * self.Hhat[1] * dT02dq1.T) + np.trace(dT03dq2 * self.Hhat[2] * dT03dq1.T) 
        M[0,2] = M[2,0] = np.trace(dT03dq3 * self.Hhat[2] * dT03dq1.T) 
        M[1,2] = M[2,1] = np.trace(dT03dq3 * self.Hhat[2] * dT03dq2.T)

        return M