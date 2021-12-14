import numpy as np
import scipy.linalg as la


class InverseDynamics:
    def __init__(self, jointVectorList, linkInertiaList, CoM_vectorList, linkMassList,
                 positionGain=900, velocityGain=5):
        self.b1 = jointVectorList[0]
        self.b2 = jointVectorList[1]
        self.b3 = jointVectorList[2]
        self.b4 = jointVectorList[3]
        self.m = linkMassList
        self.S = CoM_vectorList
        self.Hhat = []
        self.g = np.matrix([0., 0., -9.81, 0.]).T

        for i in range(3):
            self.Hhat.append(self.HhatMatrix(Ihat=linkInertiaList[i], S=CoM_vectorList[i], m=linkMassList[i]))

        self.Kp = positionGain
        self.Kv = velocityGain

    def forward(self, jointPos, jointAccel, jointVel):
        M = self.M(jointPos)
        # h = self.h(jointPos, jointVel)
        return M * jointAccel  # + h

    def solve(self, accelRef, pos, posRef, velRef, jointVel, jointPos, jacobian, diffJacobian):

        M = self.M(jointPos)
        h = self.h(jointPos, jointVel)
        g = self.g(jointPos)
        x = np.matrix(pos).T
        xd = np.matrix(posRef).T
        vd = np.matrix(velRef).T
        aRef = np.matrix(accelRef).T
        dtheta = np.matrix(jointVel).T

        v = jacobian * dtheta

        feedForwardTau = M * la.inv(jacobian) * (aRef - diffJacobian * dtheta) + h + g
        feedBackTau = jacobian.T * (self.Kv * (vd - v) + self.Kp * (xd - x))
        tau = feedBackTau + feedForwardTau

        return [tau[0,0], tau[1,0], tau[2,0]]

    def M(self, theta):
        M = np.matrix(np.zeros((3, 3)))

        q1 = theta[0]
        q2 = theta[1]
        q3 = theta[2]

        dT01dq1 = np.matrix(np.zeros((4, 4)))
        dT01dq1[1, 1] = -np.sin(q1)
        dT01dq1[1, 2] = -np.cos(q1)
        dT01dq1[2, 1] = np.cos(q1)
        dT01dq1[2, 2] = -np.sin(q1)

        dT02dq1 = np.matrix(np.zeros((4, 4)))
        dT02dq1[1, 0] = np.cos(q1) * np.sin(q2)
        dT02dq1[1, 1] = -np.sin(q1)
        dT02dq1[1, 2] = -np.cos(q1) * np.cos(q2)
        dT02dq1[1, 3] = -self.b2[2] * np.cos(q1) - self.b2[1] * np.sin(q1)
        dT02dq1[2, 0] = np.sin(q1) * np.sin(q2)
        dT02dq1[2, 1] = np.cos(q1)
        dT02dq1[2, 2] = -np.cos(q2) * np.sin(q1)
        dT02dq1[2, 3] = self.b2[1] * np.cos(q1) - self.b2[2] * np.sin(q1)

        dT02dq2 = np.matrix(np.zeros((4, 4)))
        dT02dq2[0, 0] = -np.sin(q2)
        dT02dq2[0, 2] = np.cos(q2)
        dT02dq2[1, 0] = np.cos(q2) * np.sin(q1)
        dT02dq2[1, 2] = np.sin(q1) * np.sin(q2)
        dT02dq2[2, 0] = -np.cos(q1) * np.cos(q2)
        dT02dq2[2, 2] = -np.cos(q1) * np.sin(q2)

        dT03dq1 = np.matrix(np.zeros((4, 4)))
        dT03dq1[1, 0] = np.sin(q2 + q3) * np.cos(q1)
        dT03dq1[1, 1] = -np.sin(q1)
        dT03dq1[1, 2] = -np.cos(q2 + q3) * np.cos(q1)
        dT03dq1[1, 3] = self.b3[0] * np.cos(q1) * np.sin(q2) - self.b2[1] * np.sin(q1) - self.b3[1] * np.sin(q1) - \
                        self.b3[2] * np.cos(q1) * np.cos(q2) - self.b2[2] * np.cos(q1)
        dT03dq1[2, 0] = np.sin(q2 + q3) * np.sin(q1)
        dT03dq1[2, 1] = np.cos(q1)
        dT03dq1[2, 2] = -np.cos(q2 + q3) * np.sin(q1)
        dT03dq1[2, 3] = self.b2[1] * np.cos(q1) - self.b3[1] * np.cos(q1) - self.b2[2] * np.sin(q1) - \
                        self.b3[2] * np.cos(q2) * np.sin(q1) + self.b3[0] * np.sin(q1) * np.sin(q2)

        dT03dq2 = np.matrix(np.zeros((4, 4)))
        dT03dq2[0, 0] = -np.sin(q2 + q3)
        dT03dq2[0, 2] = np.cos(q2 + q3)
        dT03dq2[0, 3] = self.b3[2] * np.cos(q2) - self.b3[0] * np.sin(q2)
        dT03dq2[1, 0] = np.cos(q2 + q3) * np.sin(q1)
        dT03dq2[1, 2] = np.sin(q2 + q3) * np.sin(q1)
        dT03dq2[1, 3] = np.sin(q1) * (self.b3[0] * np.cos(q2) + self.b3[2] * np.sin(q2))
        dT03dq2[2, 0] = -np.cos(q2 + q3) * np.cos(q1)
        dT03dq2[2, 2] = -np.sin(q2 + q3) * np.cos(q1)
        dT03dq2[2, 3] = -np.cos(q1) * (self.b3[0] * np.cos(q2) + self.b3[2] * np.sin(q2))

        dT03dq3 = np.matrix(np.zeros((4, 4)))
        dT03dq3[0, 0] = -np.sin(q2 + q3)
        dT03dq3[0, 2] = np.cos(q2 + q3)
        dT03dq3[1, 0] = np.cos(q2 + q3) * np.sin(q1)
        dT03dq3[1, 2] = np.sin(q2 + q3) * np.sin(q1)
        dT03dq3[2, 0] = -np.cos(q2 + q3) * np.cos(q1)
        dT03dq3[2, 2] = -np.sin(q2 + q3) * np.cos(q1)

        M[0, 0] = np.trace(dT01dq1 * self.Hhat[0] * dT01dq1.T) + np.trace(dT02dq1 * self.Hhat[1] * dT02dq1.T) + \
                  np.trace(dT03dq1 * self.Hhat[2] * dT03dq1.T)
        M[1, 1] = np.trace(dT02dq2 * self.Hhat[1] * dT02dq2.T) + np.trace(dT03dq2 * self.Hhat[2] * dT03dq2.T)
        M[2, 2] = np.trace(dT03dq3 * self.Hhat[2] * dT03dq3.T)

        M[0, 1] = M[1, 0] = np.trace(dT02dq2 * self.Hhat[1] * dT02dq1.T) + np.trace(dT03dq2 * self.Hhat[2] * dT03dq1.T)
        M[0, 2] = M[2, 0] = np.trace(dT03dq3 * self.Hhat[2] * dT03dq1.T)
        M[1, 2] = M[2, 1] = np.trace(dT03dq3 * self.Hhat[2] * dT03dq2.T)

        return M

    def HhatMatrix(self, Ihat, S, m):
        Hhat = np.matrix(np.zeros((4, 4)))
        Hhat[0:3, 0:3] = -Ihat

        Hhat[0, 0] = (-Ihat[0, 0] + Ihat[1, 1] + Ihat[2, 2]) / 2
        Hhat[1, 1] = (Ihat[0, 0] - Ihat[1, 1] + Ihat[2, 2]) / 2
        Hhat[2, 2] = (Ihat[0, 0] + Ihat[1, 1] - Ihat[2, 2]) / 2
        Hhat[0, 3] = Hhat[3, 0] = m * S[0]
        Hhat[1, 3] = Hhat[3, 1] = m * S[1]
        Hhat[2, 3] = Hhat[3, 2] = m * S[2]
        Hhat[3, 3] = m

        return Hhat
