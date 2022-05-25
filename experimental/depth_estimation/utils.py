import pybullet as p
import math


def createWall(dist, width):
    collisionShape = p.createCollisionShape(p.GEOM_BOX, halfExtents=[1, width/2, 1])
    visualShape = p.createVisualShape(p.GEOM_BOX, halfExtents=[1, width/2, 1])
    return p.createMultiBody(baseMass=1,
                             baseCollisionShapeIndex=collisionShape,
                             baseVisualShapeIndex=visualShape,
                             basePosition=[dist, 0, 1])


def getOnboardCamera(objId, camJoint, camInfo):
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
    return p.getCameraImage(width=400, height=300, viewMatrix=viewMat,
                            projectionMatrix=projMat,
                            renderer=p.ER_BULLET_HARDWARE_OPENGL)


def measureWidth(img):
    widths = []
    for x in range(img.shape[0]):
        width = 0
        for y in range(img.shape[1]):
            if img[x][y] == 2:
                width += 1
        widths.append(width)

    return max(widths)


def measureDist(obj1, obj2):
    pos1, _ = p.getBasePositionAndOrientation(obj1)
    pos2, _ = p.getBasePositionAndOrientation(obj2)

    # dist = math.sqrt((pos1[0] - pos2[0])**2 +
    #                  (pos1[1] - pos2[1])**2 +
    #                  (pos1[2] - pos2[2])**2)
    dist = abs(pos1[0] - pos2[0])
    return dist
