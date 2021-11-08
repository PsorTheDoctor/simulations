import pybullet as p
import random
import math

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


def getCameraView(objId, camJoint, camInfo):

    ls = p.getLinkState(objId, camJoint, computeForwardKinematics=True)
    camPos = ls[0]
    camOrn = ls[1]
    camMat = p.getMatrixFromQuaternion(camOrn)
    forwardVec = [camMat[0], camMat[3], camMat[6]]
    camUpVec = [camMat[2], camMat[5], camMat[8]]
    camTarget = [camPos[0] + forwardVec[0] * 10,
                 camPos[1] + forwardVec[1] * 10,
                 camPos[2] + forwardVec[2] * 10]
    viewMat = p.computeViewMatrix(camPos, camTarget, camUpVec)
    projMat = camInfo[3]
    p.getCameraImage(320, 200, viewMatrix=viewMat, projectionMatrix=projMat,
                     renderer=p.ER_BULLET_HARDWARE_OPENGL)


def calculateRays(numRays):

    rayFrom = []
    rayTo = []
    rayLen = 8
    rayStartLen = 0.25

    for i in range(numRays):
        angle = -0.5 * 0.25 * 2. * math.pi + 0.75 * 2. * math.pi * float(i) / numRays
        rayFrom.append([rayStartLen * math.sin(angle), rayStartLen * math.cos(angle), 0])
        rayTo.append([rayLen * math.sin(angle), rayLen * math.cos(angle), 0])

    return rayFrom, rayTo


def initLidar(objId, lidarJoint, numRays, rayFrom, rayTo):

    rayIds = []
    rayMissColor = [0, 1, 0]
    rayHitColor = [1, 0, 0]
    numThreads = 0
    results = p.rayTestBatch(rayFrom, rayTo, numThreads, parentObjectUniqueId=objId,
                             parentLinkIndex=lidarJoint)
    for i in range(numRays):
        # hitObjectUid = results[i][0]
        hitFraction = results[i][2]
        # hitPosition = results[i][3]
        if hitFraction == 1.0:
            ray = p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor,
                                     parentObjectUniqueId=objId,
                                     parentLinkIndex=lidarJoint)
        else:
            localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                          rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                          rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]

            ray = p.addUserDebugLine(rayFrom[i], localHitTo, rayHitColor,
                                     parentObjectUniqueId=objId,
                                     parentLinkIndex=lidarJoint)
        rayIds.append(ray)
    return rayIds


def updateLidar(objId, lidarJoint, numRays, rayFrom, rayTo, rayIds):

    rayMissColor = [0, 1, 0]
    rayHitColor = [1, 0, 0]
    numThreads = 0
    results = p.rayTestBatch(rayFrom, rayTo, numThreads, parentObjectUniqueId=objId,
                             parentLinkIndex=lidarJoint)
    for i in range(numRays):
        hitFraction = results[i][2]
        if hitFraction == 1.0:
            p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor,
                               replaceItemUniqueId=rayIds[i],
                               parentObjectUniqueId=objId,
                               parentLinkIndex=lidarJoint)
        else:
            localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                          rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                          rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]

            p.addUserDebugLine(rayFrom[i], localHitTo, rayHitColor,
                               replaceItemUniqueId=rayIds[i],
                               parentObjectUniqueId=objId,
                               parentLinkIndex=lidarJoint)


def removeLidar(rayIds):
    for ray in rayIds:
        p.removeUserDebugItem(ray)


def configControl(basePos, wheels, wheelVelocities, wheelDeltasTurn, wheelDeltasFwd,
                 shift, speed):

    keys = p.getKeyboardEvents()

    if ord('a') in keys:
        basePos = [basePos[0], basePos[1] - shift, basePos[2]]
    if ord('d') in keys:
        basePos = [basePos[0], basePos[1] + shift, basePos[2]]

    if p.B3G_LEFT_ARROW in keys:
        for i in range(len(wheels)):
            wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasTurn[i]
    if p.B3G_RIGHT_ARROW in keys:
        for i in range(len(wheels)):
            wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasTurn[i]
    if p.B3G_UP_ARROW in keys:
        for i in range(len(wheels)):
            wheelVelocities[i] = wheelVelocities[i] + speed * wheelDeltasFwd[i]
    if p.B3G_DOWN_ARROW in keys:
        for i in range(len(wheels)):
            wheelVelocities[i] = wheelVelocities[i] - speed * wheelDeltasFwd[i]
    # Brake
    if p.B3G_SPACE in keys:
        for i in range(len(wheels)):
            wheelVelocities[i] = 0

    return basePos, wheelVelocities


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
