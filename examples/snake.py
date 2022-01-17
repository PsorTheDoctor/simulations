import pybullet as p
import pybullet_data
import math
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.createCollisionShape(p.GEOM_PLANE)
p.createMultiBody(0, plane)

useMaximalCoordinates = True
sphereRadius = 0.5
colShapeId = p.createCollisionShape(p.GEOM_BOX,
                                    halfExtents=[0.1, sphereRadius, 0.1])

mass = 1
visualShapeId = -1
linkMasses = []
linkCollisionShapeIndices = []
linkVisualShapeIndices = []
linkPositions = []
linkOrientations = []
linkInertialFramePositions = []
linkInertialFrameOrientations = []
indices = []
jointTypes = []
axis = []

for i in range(2):
    linkMasses.append(mass)
    linkCollisionShapeIndices.append(colShapeId)
    linkVisualShapeIndices.append(-1)
    linkPositions.append([0, sphereRadius * 2.0 + 0.01, 0])
    linkOrientations.append([0, 0, 0, 1])
    linkInertialFramePositions.append([0, 0, 0])
    linkInertialFrameOrientations.append([0, 0, 0, 1])
    indices.append(i)
    jointTypes.append(p.JOINT_REVOLUTE)
    axis.append([0, 0, 1])

basePos = [0, 0, 1]
baseOrn = [0, 0, 0, 1]
sphereUid = p.createMultiBody(mass, colShapeId, visualShapeId, basePos, baseOrn,
                              linkMasses=linkMasses,
                              linkCollisionShapeIndices=linkCollisionShapeIndices,
                              linkVisualShapeIndices=linkVisualShapeIndices,
                              linkPositions=linkPositions,
                              linkOrientations=linkOrientations,
                              linkInertialFramePositions=linkInertialFramePositions,
                              linkInertialFrameOrientations=linkInertialFrameOrientations,
                              linkParentIndices=indices,
                              linkJointTypes=jointTypes,
                              linkJointAxis=axis,
                              useMaximalCoordinates=useMaximalCoordinates)

p.setGravity(0, 0, -9.81)
p.setRealTimeSimulation(True)

anisotropicFriction = [1, 0.01, 0.01]
p.changeDynamics(sphereUid, -1, lateralFriction=2, anisotropicFriction=anisotropicFriction)
numJoints = p.getNumJoints(sphereUid)
for i in range(numJoints):
    p.getJointInfo(sphereUid, i)
    p.changeDynamics(sphereUid, i, lateralFriction=2, anisotropicFriction=anisotropicFriction)

dt = 1. / 240.
wavePeriod = 0.5
waveLength = 1.5
waveAmplitude = 0.4
waveFront = 0.0
segmentLength = sphereRadius * 2.0
forward = True

while True:
    pos = p.getBasePositionAndOrientation(sphereUid)[0]
    p.resetDebugVisualizerCamera(cameraDistance=3, cameraYaw=90, cameraPitch=-41,
                                 cameraTargetPosition=pos)
    scaleStart = 1.0

    if waveFront < segmentLength * 4.0:
        scaleStart = waveFront / segmentLength * 4.0

    # Moving a sin wave down the body of the snake.
    for joint in range(numJoints):
        segment = joint
        phase = (waveFront - (segment + 1) * segmentLength) / waveLength
        phase -= math.floor(phase)
        phase *= math.pi * 2.0

        # Map phase to curvature
        targetPos = math.sin(phase) * scaleStart * waveAmplitude

        p.setJointMotorControl2(sphereUid, joint, p.POSITION_CONTROL,
                                targetPosition=targetPos, force=10)

    # Wave keeps track of where the wave is in time.
    waveFront += dt / wavePeriod * waveLength
    p.stepSimulation()
    time.sleep(dt)
