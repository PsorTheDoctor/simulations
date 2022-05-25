import numpy as np
from robots.quadruped.robot import Quadruped
import time


def creepingGait():
    qdrp = Quadruped(timeStep=1./240., initialCoM_height=0.3, startPos=[0,0,0.55],
                     startOrn=[0.,0.,0.], maxForce=12, robotPath='quadruped.urdf')

    targetPosRF = np.array([0.2, -0.11, -0.2])
    targetPosRH = np.array([-0.2, -0.11, -0.2])
    targetPosLF = np.array([0.2, 0.11, -0.2])
    targetPosLH = np.array([-0.2, 0.11, -0.2])

    dp = 0.001
    for _ in range(3):
        # Step 1
        for _ in np.arange(0, 0.1, 0.001):
            targetPosLH[0] += 0.00005
            targetPosLH[2] += 0.0001

            LHjointPositions = qdrp.inverseKinematics(targetPosLH, targetLeg=qdrp.legLH)
            qdrp.legLH.setJointPositions(LHjointPositions)
            qdrp.oneStep()
        time.sleep(3)

        # Step 2
        for _ in np.arange(0, 0.1, 0.001):
            targetPosLF[2] += dp
            targetPosRF[2] += dp
            targetPosRH[2] += dp

            LFjointPositions = qdrp.inverseKinematics(targetPosLF, targetLeg=qdrp.legLF)
            RFjointPositions = qdrp.inverseKinematics(targetPosRF, targetLeg=qdrp.legRF)
            RHjointPositions = qdrp.inverseKinematics(targetPosRH, targetLeg=qdrp.legRH)

            qdrp.legLF.setJointPositions(LFjointPositions)
            qdrp.legRF.setJointPositions(RFjointPositions)
            qdrp.legRH.setJointPositions(RHjointPositions)
            qdrp.oneStep()

        # Step 3
        for _ in np.arange(0, 0.1, 0.001):
            pass

        # Step 4
        for _ in np.arange(0, 0.1, 0.001):
            # targetPosLF = np.array([b, c, height])
            targetPosLF[0] += dp

            LFjointPositions = qdrp.inverseKinematics(targetPosLF, targetLeg=qdrp.legLF)
            qdrp.legLF.setJointPositions(LFjointPositions)
            qdrp.oneStep()

    qdrp.disconnect()


def squat():
    targetPosRF = np.array([0.2, -0.11, -0.2])
    targetPosRH = np.array([-0.2, -0.11, -0.2])
    targetPosLF = np.array([0.2, 0.11, -0.2])
    targetPosLH = np.array([-0.2, 0.11, -0.2])

    qdrp = Quadruped(timeStep=1./240., initialCoM_height=0.3, startPos=[0,0,0.55],
                     startOrn=[0.,0.,0.], maxForce=12, robotPath='quadruped.urdf')

    # print(qdrp.legLF.getJointPositions())
    # print(qdrp.legRF.getJointPositions())
    # print(qdrp.legLH.getJointPositions())
    # print(qdrp.legRH.getJointPositions())

    dp = 0.001
    for _ in range(3):
        for _ in range(100):
            targetPosLF[2] += dp
            targetPosRF[2] += dp
            targetPosLH[2] += dp
            targetPosRH[2] += dp

            LFjointPositions = qdrp.inverseKinematics(targetPosLF, targetLeg=qdrp.legLF)
            RFjointPositions = qdrp.inverseKinematics(targetPosRF, targetLeg=qdrp.legRF)
            LHjointPositions = qdrp.inverseKinematics(targetPosLH, targetLeg=qdrp.legLH)
            RHjointPositions = qdrp.inverseKinematics(targetPosRH, targetLeg=qdrp.legRH)

            qdrp.legLF.setJointPositions(LFjointPositions)
            qdrp.legRF.setJointPositions(RFjointPositions)
            qdrp.legLH.setJointPositions(LHjointPositions)
            qdrp.legRH.setJointPositions(RHjointPositions)
            qdrp.oneStep()

        for _ in range(100):
            targetPosLF[2] -= dp
            targetPosRF[2] -= dp
            targetPosLH[2] -= dp
            targetPosRH[2] -= dp

            LFjointPositions = qdrp.inverseKinematics(targetPosLF, targetLeg=qdrp.legLF)
            RFjointPositions = qdrp.inverseKinematics(targetPosRF, targetLeg=qdrp.legRF)
            LHjointPositions = qdrp.inverseKinematics(targetPosLH, targetLeg=qdrp.legLH)
            RHjointPositions = qdrp.inverseKinematics(targetPosRH, targetLeg=qdrp.legRH)

            qdrp.legLF.setJointPositions(LFjointPositions)
            qdrp.legRF.setJointPositions(RFjointPositions)
            qdrp.legLH.setJointPositions(LHjointPositions)
            qdrp.legRH.setJointPositions(RHjointPositions)
            qdrp.oneStep()

    qdrp.disconnect()


if __name__ == '__main__':
    squat()
