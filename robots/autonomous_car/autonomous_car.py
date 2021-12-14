import pybullet as p
import math
from time import time

p.connect(p.GUI)
p.resetSimulation()
p.setGravity(0, 0, -9.81)
useRealTimeSim = False

p.setTimeStep(1./120.)
p.setRealTimeSimulation(useRealTimeSim)

track = p.loadSDF('f10_racecar/meshes/barca_track.sdf', globalScaling=True)
car = p.loadURDF('f10_racecar/racecar_differential.urdf', [0, 0, .3])

for wheel in range(p.getNumJoints(car)):
    print('Joint[', wheel, ']=', p.getJointInfo(car, wheel))
    p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL, targetVelocity=0, force=0)

c = p.createConstraint(car, 9, car, 11, jointType=p.JOINT_GEAR, jointAxis=[0,1,0],
                       parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 10, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0,1,0],
                       parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 9, car, 13, jointType=p.JOINT_GEAR, jointAxis=[0,1,0],
                       parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 16, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0,1,0],
                       parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
p.changeConstraint(c, gearRatio=1, maxForce=10000)

c = p.createConstraint(car, 16, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0,1,0],
                       parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 17, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0,1,0],
                       parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
p.changeConstraint(c, gearRatio=-1, maxForce=10000)

c = p.createConstraint(car, 1, car, 18, jointType=p.JOINT_GEAR, jointAxis=[0,1,0],
                       parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

c = p.createConstraint(car, 3, car, 19, jointType=p.JOINT_GEAR, jointAxis=[0,1,0],
                       parentFramePosition=[0,0,0], childFramePosition=[0,0,0])
p.changeConstraint(c, gearRatio=-1, gearAuxLink=15, maxForce=10000)

wheels = [8, 15]
steering = [0, 2]
hokuyo_joint = 4
zed_camera_joint = 5

targetVelocitySlider = p.addUserDebugParameter('Velocity', -50, 50, 0)
maxForceSlider = p.addUserDebugParameter('Max force', 0, 50, 20)
steeringSlider= p.addUserDebugParameter('Steering', -1, 1, 0)

replaceLines = True
numRays = 100
rayFrom = []
rayTo = []
rayIds = []
rayHitColor = [1,0,0]
rayMissColor = [0,1,0]
rayLen = 8
rayStartLen = 0.25

for i in range(numRays):
    angle = -0.5*0.25*2.*math.pi+0.75*2.*math.pi*float(i)/numRays

    rayFrom.append([rayStartLen * math.sin(angle), rayStartLen * math.cos(angle), 0])
    rayTo.append([rayLen * math.sin(angle), rayLen * math.cos(angle), 0])
    if replaceLines:
        rayIds.append(p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor, parentObjectUniqueId=car,
                                         parentLinkIndex=hokuyo_joint))
    else:
        rayIds.append(-1)


def getCarYaw(car):
    carPos, carOrn = p.getBasePositionAndOrientation(car)
    carEuler = p.getEulerFromQuaternion(carOrn)
    carYaw = carEuler[2] * 360 / (2. * math.pi) - 90
    return carYaw


prevCarYaw = getCarYaw(car)

lineId = p.addUserDebugLine([0,0,0], [0,0,1], [1,0,0])
lineId2 = p.addUserDebugLine([0,0,0], [0,0,1], [1,0,0])
lineId3 = p.addUserDebugLine([0,0,0], [0,0,1], [1,0,0])
print('lineId=', lineId)

camInfo = p.getDebugVisualizerCamera()
lastTime = time()
lastControlTime = time()
lastLidarTime = time()

frame = 0
while True:
    nowTime = time()
    # Render camera at 10 Hz
    if nowTime - lastTime > .1:
        ls = p.getLinkState(car, zed_camera_joint, computeForwardKinematics=True)
        camPos = ls[0]
        camOrn = ls[1]
        camMat = p.getMatrixFromQuaternion(camOrn)
        upVector = [0, 0, 1]
        forwardVec = [camMat[0], camMat[3], camMat[6]]
        camUpVec = [camMat[2], camMat[5], camMat[8]]
        camTarget = [camPos[0] + forwardVec[0] * 10,
                     camPos[1] + forwardVec[1] * 10,
                     camPos[2] + forwardVec[2] * 10]
        camUpTarget = [camPos[0] + camUpVec[0], camPos[1] + camUpVec[1], camPos[2] + camUpVec[2]]
        viewMat = p.computeViewMatrix(camPos, camTarget, camUpVec)
        projMat = camInfo[3]
        p.getCameraImage(320, 200, viewMatrix=viewMat, projectionMatrix=projMat,
                         renderer=p.ER_BULLET_HARDWARE_OPENGL)
        lastTime = nowTime

    nowControlTime = time()
    nowLidarTime = time()
    if nowLidarTime - lastLidarTime > .3:
        numThreads = 0
        results = p.rayTestBatch(rayFrom, rayTo, numThreads, parentObjectUniqueId=car,
                                 parentLinkIndex=hokuyo_joint)
        for i in range(numRays):
            hitObjectUid = results[i][0]
            hitFraction = results[i][2]
            hitPosition = results[i][3]
            if hitFraction == 1.0:
                p.addUserDebugLine(rayFrom[i], rayTo[i], rayMissColor,
                                   replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car,
                                   parentLinkIndex=hokuyo_joint)
            else:
                localHitTo = [rayFrom[i][0] + hitFraction * (rayTo[i][0] - rayFrom[i][0]),
                              rayFrom[i][1] + hitFraction * (rayTo[i][1] - rayFrom[i][1]),
                              rayFrom[i][2] + hitFraction * (rayTo[i][2] - rayFrom[i][2])]
                p.addUserDebugLine(rayFrom[i], localHitTo, rayHitColor,
                                   replaceItemUniqueId=rayIds[i],
                                   parentObjectUniqueId=car,
                                   parentLinkIndex=hokuyo_joint)
        lastLidarTime = nowLidarTime

    # Control at 100 Hz
    if nowControlTime - lastControlTime > .01:
        carPos, carOrn = p.getBasePositionAndOrientation(car)

        yaw = camInfo[8]
        pitch = camInfo[9]
        distance = camInfo[10]
        targetPos = camInfo[5]
        camFwd = camInfo[5]
        carYaw = getCarYaw(car)

        if carYaw - prevCarYaw > 45:
            yaw += 360
        if carYaw - prevCarYaw < -45:
            yaw -= 360
        prevCarYaw = carYaw

        # diffYaw = (carYaw - yaw) * 0.03
        maxForce = p.readUserDebugParameter(maxForceSlider)
        targetVelocity = p.readUserDebugParameter(targetVelocitySlider)
        steeringAngle = p.readUserDebugParameter(steeringSlider)

        for wheel in wheels:
            p.setJointMotorControl2(car, wheel, p.VELOCITY_CONTROL,
                                    targetVelocity=targetVelocity,
                                    force=maxForce)
        for steer in steering:
            p.setJointMotorControl2(car, steer, p.VELOCITY_CONTROL,
                                    targetPosition=-steeringAngle)

        if not useRealTimeSim:
            frame += 1
            p.stepSimulation()
        lastControlTime = nowControlTime