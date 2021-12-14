import numpy as np
from robots.biped.robot import Biped
from robots.biped.walking import PreviewControl


def stand_up():
    biped = Biped()
    CoM_height = 0.45

    targetRPY = [0.0, 0.0, 0.0]
    targetPosL = [0.0, 0.065, -CoM_height]
    targetPosR = [0.0, -0.065, -CoM_height]
    biped.positionInitialize(initializeTime=0.2)

    while True:
        posL = biped.inverseKinematics(targetPosL, targetRPY, biped.L)
        posR = biped.inverseKinematics(targetPosR, targetRPY, biped.R)
        biped.setLeftLegJointPositions(posL)
        biped.setRightLegJointPositions(posR)
        biped.oneStep()


def walk():
    biped = Biped()
    # CoM_height = 0.45
    stride = 0.1
    # CoM_to_body = np.array([0.0, 0.0, 0.0])

    targetRPY = [0.0, 0.0, 0.0]
    pre = PreviewControl(Tsup_time=0.3, Tdl_time=0.1, previewStepNum=190)
    biped.positionInitialize(initializeTime=0.2)
    CoM_trajectory = np.empty((0, 3), float)

    trjR_log = np.empty((0, 3), float)
    trjL_log = np.empty((0, 3), float)
    walkingCycle = 100
    supPoint = np.array([0., 0.065])

    for w in range(walkingCycle):
        # Generates one cycle trajectory
        CoM_trj, footTrjL, footTrjR = pre.footPrintAndCoM_trajectoryGenerator(inputTargetZMP=supPoint,
                                                                              inputFootPrint=supPoint)
        CoM_trajectory = np.vstack((CoM_trajectory, CoM_trj))
        trjR_log = np.vstack((trjR_log, footTrjR))
        trjL_log = np.vstack((trjL_log, footTrjL))

        CoM_len = len(CoM_trj)
        for i in range(CoM_len):
            targetPosR = footTrjR[i] - CoM_trj[i]
            targetPosL = footTrjL[i] - CoM_trj[i]

            posR = biped.inverseKinematics(targetPosR, targetRPY, biped.R)
            posL = biped.inverseKinematics(targetPosL, targetRPY, biped.L)
            biped.setLeftLegJointPositions(posL)
            biped.setRightLegJointPositions(posR)
            biped.oneStep()

        supPoint[0] += stride
        supPoint[1] = -supPoint[1]

    biped.disconnect()


if __name__ == '__main__':
    stand_up()
