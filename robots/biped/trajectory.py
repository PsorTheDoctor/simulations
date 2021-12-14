import numpy as np
import scipy.linalg as la


def trajectoryGenerator(startPointVector, endPointVector, startVelocityVector_xy, endVelocityVector,
                        zheight, startTime, endTime, dt):

    x = trajectoryGenerator_xy(startPointVector[0], endPointVector[0], startVelocityVector_xy[0], endVelocityVector[0],
                               startTime, endTime, dt)
    y = trajectoryGenerator_xy(startPointVector[1], endPointVector[1], startVelocityVector_xy[1], endVelocityVector[1],
                               startTime, endTime, dt)
    z = trajectoryGenerator_z(zheight, startPointVector[2], endPointVector[2], endVelocityVector[2],
                              startTime, endTime, dt)

    return np.vstack((x, y, z)).T


def trajectoryGenerator_xy(startPoint, endPoint, startVel, endVel, startTime, endTime, dt):

    timeVec = np.arange(startTime, endTime, dt)

    A = np.matrix([startPoint, endPoint, startVel, endVel]).T

    B = np.matrix([[startTime**3, startTime**2, startTime, 1],
                  [endTime**3, endTime**2, endTime, 1],
                  [3 * (startTime**2), 2 * startTime, 1, 0],
                  [3 * (endTime**2), 2 * endTime, 1, 0]])

    C = la.inv(B) * A

    x = C[0] * (timeVec**3) + C[1] * (timeVec**2) + C[2] * timeVec + C[3]
    return np.array(x)[0]


def trajectoryGenerator_z(zheight, startPoint, endPoint, endVel, startTime, endTime, dt):

    heightTime = ((endTime - startTime) / 2) + startTime
    zh = startPoint + zheight
    timeVec = np.arange(startTime, endTime, dt)

    A = np.matrix([zh, endPoint, startPoint, endVel]).T

    B = np.matrix([[heightTime**3, heightTime**2, heightTime, 1],
                  [endTime**3, endTime**2, endTime, 1],
                  [startTime**3, startTime**2, startTime, 1],
                  [3 * (endTime**2), 2 * endTime, 1, 0]])

    C = la.inv(B) * A

    z = C[0] * (timeVec**3) + C[1] * (timeVec**2) + C[2] * timeVec + C[3]
    return np.array(z)[0]
