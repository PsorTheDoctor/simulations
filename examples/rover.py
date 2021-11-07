import pybullet as p
import pybullet_data
import numpy as np
import math
import time
from utils import utils

### SETUP
client = p.connect(p.SHARED_MEMORY)
if client < 0:
    p.connect(p.GUI)
# Get models
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setPhysicsEngineParameter(enableConeFriction=False)
# Disable rendering during creation
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, False)

### LOAD MODELS
utils.createTerrain()

# Load mobile platform
husky = p.loadURDF('husky/husky.urdf', [0.290388, 0.329902, -0.310270],
                   [0.002328, -0.000984, 0.996491, 0.083659])
for i in range(p.getNumJoints(husky)):
    print(p.getJointInfo(husky, i))

# Load manipulator
armId = p.loadURDF('kuka_iiwa/model_free_base.urdf', 0.193749, 0.345564, 0.120208, 0.002327,
                   -0.000988, 0.996491, 0.083659)
jointPositions = [3.559609, 0.411182, 0.862129, 1.744441, 0.077299, -1.129685, 0.006001]

numJoints = p.getNumJoints(armId)
for jointId in range(numJoints):
    p.resetJointState(armId, jointId, jointPositions[jointId])

armEndEffectorId = 6  # because it has 7 axes

# Put arm on a top of husky
cid = p.createConstraint(husky, -1, armId, -1, p.JOINT_FIXED,
                         [0, 0, 0], [0, 0, 0], [0., 0., -.5], [0, 0, 0, 1])

### GET CAMERA VIEW
width = 128
height = 128
fov = 60
aspect = width / height
near = 0.02
far = 1

viewMatrix = p.computeViewMatrix([0, 0, 0], [0, 0, 0], [1, 0, 0])
projectionMatrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

images = p.getCameraImage(width, height, viewMatrix, projectionMatrix, shadow=True,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
rgb_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
depth_buffer_opengl = np.reshape(images[3], [width, height])
depth_opengl = far * near / (far - (far - near)) * depth_buffer_opengl
seg_opengl = np.reshape(images[4], [width, height]) * 1. / 255.
time.sleep(1)

### DEFINE CONSTANTS
baseOrn = [0, 0, 0, 1]

# lower limits for null space
ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
# upper limits for null space
ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
# joint ranges for null space
jr = [5.8, 4, 5.8, 4, 5.8, 4, 6]
# restposes for null space
rp = [0, 0, 0, 0.5 * math.pi, 0, -math.pi * 0.5 * 0.66, 0]
# joint damping coefficients
jd = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]

for i in range(numJoints):
    p.resetJointState(armId, i, rp[i])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, True)
p.setGravity(0, 0, -9.81)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = False
useNullSpace = False
useOrientation = False
useSimulation = True
useRealTimeSimulation = True
p.setRealTimeSimulation(useRealTimeSimulation)
trailDuration = 15
basePos = [0, 0, 0]
ang = 0

wheels = [2, 3, 4, 5]
wheelVelocities = [0, 0, 0, 0]
wheelDeltasTurn = [1, -1, 1, -1]
wheelDeltasFwd = [1, 1, 1, 1]

### SIMULATION LOOP
while True:
    keys = p.getKeyboardEvents()
    shift = 0.001
    speed = 0.1
    for k in keys:
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

    baseOrn = p.getQuaternionFromEuler([0, 0, ang])
    for i in range(len(wheels)):
        p.setJointMotorControl2(husky,
                                wheels[i],
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheelVelocities[i],
                                force=100)
    if useRealTimeSimulation:
        t = time.time()
    else:
        t += 0.001

    if useSimulation and not useRealTimeSimulation:
        p.stepSimulation()

    # Drawing a circle
    pos = [0.2 * math.cos(t), 0, 0.2 * math.sin(t) + 0.7]
    # End effector points down
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    if useNullSpace:
        if useOrientation:
            jointPoses = p.calculateInverseKinematics(armId, armEndEffectorId,
                                                      pos, orn, ll, ul, jr, rp)
        else:
            jointPoses = p.calculateInverseKinematics(armId,
                                                      armEndEffectorId,
                                                      pos,
                                                      lowerLimits=ll,
                                                      upperLimits=ul,
                                                      jointRanges=jr,
                                                      restPoses=rp)
    else:
        if useOrientation:
            jointPoses = p.calculateInverseKinematics(armId,
                                                      armEndEffectorId,
                                                      pos,
                                                      orn,
                                                      jointDamping=jd)
        else:
            threshold = 0.001
            maxIter = 100
            jointPoses = utils.calculateInverseKinematics(armId, armEndEffectorId, pos,
                                                          threshold, maxIter)
    if useSimulation:
        for i in range(numJoints):
            p.setJointMotorControl2(bodyIndex=armId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=50,
                                    positionGain=1,
                                    velocityGain=0.1)
    else:
        # Reset the joint state
        for i in range(numJoints):
            p.resetJointState(armId, i, jointPoses[i])

    ls = p.getLinkState(armId, armEndEffectorId)
    if hasPrevPose:
        p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = True
