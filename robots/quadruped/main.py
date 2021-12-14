import numpy as np
from robots.quadruped.robot import Quadruped


def push_ups():
    targetPosRF = np.array([0.2, -0.11, -0.2])
    targetPosRH = np.array([-0.2, -0.11, -0.2])
    targetPosLF = np.array([0.2, 0.11, -0.2])
    targetPosLH = np.array([-0.2, 0.11, -0.2])

    qdrp = Quadruped(timeStep=1./240., initialCoM_height=0.3, startPos=[0,0,0.55],
                     startOrn=[0.,0.,0.], maxForce=12, robotPath='quadruped.urdf')
    dp = 0.001
    for _ in range(3):
        for _ in np.arange(0, 0.1, 0.001):
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

        for _ in np.arange(0, 0.1, 0.001):
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
    push_ups()
