import pybullet as p
import pybullet_data
import numpy as np
import time
import robots.quadruped.transform as tf
import scipy.linalg as la
from .inverse_dynamics import InverseDynamics


class Robot:
    def __init__(self, timeStep, robotPath, startPos, startOrn, maxForce,
                 controlMode=p.POSITION_CONTROL, planePath='plane.urdf'):
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        self.planeId = p.loadURDF(planePath)
        self.robotId = p.loadURDF(robotPath, startPos, p.getQuaternionFromEuler(startOrn))
        self.controlMode = controlMode
        self.numJoint = p.getNumJoints(self.robotId)
        self.jointIdList = list(range(self.numJoint))

        self.maxForce = maxForce
        self.maxForceList = [maxForce] * 12
        self.timeStep = timeStep
        self.bodyLinkId = -1

        robotPos, _ = p.getBasePositionAndOrientation(self.robotId)
        p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=180, cameraPitch=-10,
                                     cameraTargetPosition=robotPos)

    def getTimeStep(self):
        return self.timeStep

    def getBodyLinkState(self):
        return p.getLinkState(self.robotId, self.bodyLinkId)

    def getEuler(self):
        _, qua = p.getBasePositionAndOrientation(self.robotId)
        return p.getEulerFromQuaternion(qua)

    def getQuaternion(self):
        _, orn = p.getBasePositionAndOrientation(self.robotId)
        return orn

    def getRobotPosition(self):
        pos, _ = p.getBasePositionAndOrientation(self.robotId)
        return pos

    def resetRobotPositionAndOrientation(self, pos, orn):
        p.resetBasePositionAndOrientation(self.robotId, pos, orn)

    def oneStep(self):
        p.stepSimulation()
        time.sleep(self.timeStep)


class Leg:
    def __init__(self, robotId, name, firstJointOrigin, DoF, idIndices, axisMatrix,
                 maxForce=12, maxVel=10, mode=p.POSITION_CONTROL):
        self.robotId = robotId
        self.firstJointOrigin = firstJointOrigin
        self.DoF = DoF
        self.axisMatrix = axisMatrix
        self.name = name
        self.idIndices = idIndices
        self.maxVel = maxVel
        self.maxForceList = [maxForce for _ in range(DoF)]
        self.controlMode = mode
        self.linkInertiaList = [np.matrix(np.zeros((3, 3))),
                                np.matrix([[0.002312157023, 0., 0.],
                                           [0., 0.002476174735, 0.],
                                           [0., 0., 0.00028213453]]),
                                np.matrix([[0.000664235357, 0., 0.],
                                           [0., 0.000664515268, 0.],
                                           [0., 0., 0.000019650311]])]
        self.jointPosVectors = [firstJointOrigin,
                                np.zeros(3),
                                np.array([0, 0, -0.18]),
                                np.array([0, 0, -0.18])]

        self.inverseDynamics = InverseDynamics(jointVectorList=self.jointPosVectors,
                                               linkInertiaList=self.linkInertiaList,
                                               CoM_vectorList=[np.zeros(3), np.array([0.,0.,-0.033267]),
                                                              np.array([0.,0.,-0.155989])],
                                               linkMassList=[0., 0.376687, 0.140882])

    def getJointPositions(self):
        jointStates = p.getJointStates(self.robotId, jointIndices=self.idIndices)
        jointPositions = [jointStates[i][0] for i in range(self.DoF)]
        return jointPositions

    def getJointVelocity(self):
        jointStates = p.getJointStates(self.robotId, jointIndices=self.idIndices)
        jointVelocity = [jointStates[i][1] for i in range(self.DoF)]
        return jointVelocity

    def setJointPositions(self, targetPositions):
        p.setJointMotorControlArray(self.robotId, jointIndices=self.idIndices, controlMode=self.controlMode,
                                    forces=self.maxForceList, targetPositions=targetPositions)

    def torqueControlModeEnable(self):
        p.setJointMotorControlArray(self.robotId, jointIndices=self.idIndices, controlMode=p.VELOCITY_CONTROL,
                                    forces=[0 for _ in range(self.DoF)])
        self.controlMode = p.TORQUE_CONTROL

    def setTorqueArray(self, torque):
        p.setJointMotorControlArray(self.robotId, jointIndices=self.idIndices, controlMode=self.controlMode,
                                    forces=torque)


class Quadruped(Robot):
    def __init__(self, timeStep, robotPath, initialCoM_height, startPos=[0,0,0.55], startOrn=[0.,0.,0.],
                 maxForce=9.0, controlMode=p.POSITION_CONTROL, planePath='plane.urdf'):
        super().__init__(timeStep=timeStep, robotPath=robotPath, startPos=startPos, startOrn=startOrn,
                         maxForce=maxForce, controlMode=controlMode, planePath=planePath)
        self.L1 = 0.18
        self.L2 = 0.18
        self.a = np.array([[1, 0, 0],
                           [0, 1, 0],
                           [0, 1, 0]])
        # RF: Right Forward
        # RH: Right Hind
        # LF: Left Forward
        # LH: Left Hind
        self.legRF = Leg(self.robotId, "RF", np.array([0.2, -0.11, 0.]), 3, [0, 1, 2], self.a)
        self.legLF = Leg(self.robotId, "LF", np.array([0.2, 0.11, 0.]), 3, [3, 4, 5], self.a)
        self.legRH = Leg(self.robotId, "RH", np.array([-0.2, -0.11, 0.]), 3, [6, 7, 8], self.a)
        self.legLH = Leg(self.robotId, "LH", np.array([-0.2, 0.11, 0.]), 3, [9, 10, 11], self.a)

        self.LEG_NUM = 4
        self.E = np.eye(3)
        self.jacobi_lambda = 1.

        self.initializer(initialFootPrints=np.array([np.hstack((self.legLF.firstJointOrigin[0:2], -initialCoM_height)),
                                                     np.hstack((self.legLH.firstJointOrigin[0:2], -initialCoM_height)),
                                                     np.hstack((self.legRF.firstJointOrigin[0:2], -initialCoM_height)),
                                                     np.hstack((self.legRH.firstJointOrigin[0:2], -initialCoM_height))]),
                         initialPosition=startPos,
                         initialOrientation=p.getQuaternionFromEuler(startOrn))

        self.inertiaTensor = np.matrix(np.zeros((3, 3)))
        self.inertiaTensor[0, 0] = 0.017409405067
        self.inertiaTensor[1, 1] = 0.043070296402
        self.inertiaTensor[2, 2] = 0.052179256932

    def forwardKinematics(self, jointPositions, targetLeg, fullReturn=False):
        abadJoint = jointPositions[0]
        hipJoint = jointPositions[1]
        kneeJoint = jointPositions[2]

        zero_v = np.zeros(3)

        T_0_1 = tf.getTransFromRp(tf.rodriguesEquation(self.E, targetLeg.axisMatrix[0], abadJoint), targetLeg.firstJointOrigin)
        T_0_2 = T_0_1.dot(tf.getTransFromRp(tf.rodriguesEquation(self.E, targetLeg.axisMatrix[1], hipJoint), zero_v))
        T_0_3 = T_0_2.dot(tf.getTransFromRp(tf.rodriguesEquation(self.E, targetLeg.axisMatrix[2], kneeJoint), [0, 0, -self.L1]))
        T_0_4 = T_0_3.dot(tf.getTransFromRp(np.eye(3, 3), [0, 0, -self.L2]))

        if fullReturn:
          return T_0_1, T_0_2, T_0_3, T_0_4
        else:
          return tf.getRotationAndPositionFromT(T_0_4)

    def inverseKinematics(self, position_ref, targetLeg):
        q = targetLeg.getJointPositions()
        _, position = self.forwardKinematics(q, targetLeg)
        dp = position_ref - position

        dq = self.jacobi_lambda * la.inv(self.jacobian(q, targetLeg)).dot(dp)
        return q + dq

    def jacobian(self, q, targetLeg):
      T_0_E = self.forwardKinematics(q, targetLeg, fullReturn=True)

      R = [tf.getRotationFromT(T_0_E[i]) for i in range(len(T_0_E))]
      p = [tf.getPositionFromT(T_0_E[i]) for i in range(len(T_0_E))]

      wa = [R[i].dot(targetLeg.axisMatrix[i]) for i in range(targetLeg.DoF)]

      J = np.vstack((np.cross(wa[0], (p[-1]-p[0])), np.cross(wa[1], (p[-1]-p[1])), np.cross(wa[2], (p[-1]-p[2])))).T
      return J

    def initializer(self, initialFootPrints, initialPosition, initialOrientation,
                    initialJointPosition=np.array([0.,0.2,-0.4]), initializeTime=1.):

        for _ in np.arange(0, initializeTime / self.timeStep, 1):
            self.resetRobotPositionAndOrientation(initialPosition, initialOrientation)

            self.legRF.setJointPositions(initialJointPosition)
            self.legLF.setJointPositions(initialJointPosition)
            self.legRH.setJointPositions(initialJointPosition)
            self.legLH.setJointPositions(initialJointPosition)
            self.oneStep()

        for _ in np.arange(0, initializeTime / self.timeStep, 1):
            posLF = self.inverseKinematics(initialFootPrints[0], self.legLF)
            posLH = self.inverseKinematics(initialFootPrints[1], self.legLH)
            posRF = self.inverseKinematics(initialFootPrints[2], self.legRF)
            posRH = self.inverseKinematics(initialFootPrints[3], self.legRH)

            self.legRF.setJointPositions(posLF)
            self.legLF.setJointPositions(posLH)
            self.legRH.setJointPositions(posRF)
            self.legLH.setJointPositions(posRH)
            self.oneStep()

    def disconnect(self):
        p.disconnect()
