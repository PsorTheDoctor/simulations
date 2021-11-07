import pybullet as p
import random

def createTerrain():

    random.seed(10)
    heightPerturbationRange = 0.05
    numRows = 256
    numColumns = 256
    heightfieldData = [0] * numRows * numColumns

    for j in range(int(numColumns / 2)):
        for i in range(int(numRows / 2)):
            height = random.uniform(0, heightPerturbationRange)
            heightfieldData[2 * i + 2 * j * numRows] = height
            heightfieldData[2 * i + 1 + 2 * j * numRows] = height
            heightfieldData[2 * i + (2 * j + 1) * numRows] = height
            heightfieldData[2 * i + 1 + (2 * j + 1) * numRows] = height

    terrainShape = p.createCollisionShape(shapeType = p.GEOM_HEIGHTFIELD,
                                          meshScale=[.05, .05, 1],
                                          heightfieldTextureScaling=(numRows-1)/2,
                                          heightfieldData=heightfieldData,
                                          numHeightfieldRows=numRows,
                                          numHeightfieldColumns=numColumns)
    terrain  = p.createMultiBody(0, terrainShape)
    p.resetBasePositionAndOrientation(terrain, [0, 0, -0.3], [0, 0, 0, 1])
    p.changeVisualShape(terrain, -1, rgbaColor=[0.5, 1, 0.5, 1])


def calculateInverseKinematics(armId, endEffectorId, targetPos, threshold, maxIter):

    numJoints = p.getNumJoints(armId)
    closeEnough = False
    iter = 0
    jointPoses = None

    while not closeEnough and iter < maxIter:
        jointPoses = p.calculateInverseKinematics(armId, endEffectorId, targetPos)
        for i in range(numJoints):
            p.resetJointState(armId, i, jointPoses[i])
        ls = p.getLinkState(armId, endEffectorId)
        newPos = ls[4]
        diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
        dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
        closeEnough = dist2 < threshold
        iter += 1
    return jointPoses
