import pybullet as p
import pybullet_data
import math
import time

client = p.connect(p.SHARED_MEMORY)
if client < 0:
    p.connect(p.GUI)

p.setPhysicsEngineParameter(enableConeFriction=False)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF('plane.urdf', [0, 0, -0.3])
husky = p.loadURDF('husky/husky.urdf', [0.290388, 0.329902, -0.310270],
                   [0.002328, -0.000984, 0.996491, 0.083659])
for i in range(p.getNumJoints(husky)):
    print(p.getJointInfo(husky, i))

armId = p.loadURDF('kuka_iiwa/model_free_base.urdf', 0.193749, 0.345564, 0.120208, 0.002327,
                   -0.000988, 0.996491, 0.083659)
jointPositions = [3.559609, 0.411182, 0.862129, 1.744441, 0.077299, -1.129685, 0.006001]
for jointId in range(p.getNumJoints(armId)):
    p.resetJointState(armId, jointId, jointPositions[jointId])

# Put arm on a top of husky
cid = p.createConstraint(husky, -1, armId, -1, p.JOINT_FIXED,
                         [0, 0, 0], [0, 0, 0], [0., 0., -.5], [0, 0, 0, 1])
# baseOrn = p.getQuaternionFromEuler([3.1415, 0, 0.3])
baseOrn = [0, 0, 0, 1]

numJoints = p.getNumJoints(armId)
armEndEffectorId = 6

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


def calculateInverseKinematics(armId, endEffectorId, targetPos, threshold, maxIter):
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


wheels = [2, 3, 4, 5]
wheelVelocities = [0, 0, 0, 0]
wheelDeltasTurn = [1, -1, 1, -1]
wheelDeltasFwd = [1, 1, 1, 1]

while True:
    keys = p.getKeyboardEvents()
    shift = 0.01
    speed = 1.0
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

    baseOrn = p.getQuaternionFromEuler([0, 0, ang])
    for i in range(len(wheels)):
        p.setJointMotorControl2(husky,
                                wheels[i],
                                p.VELOCITY_CONTROL,
                                targetVelocity=wheelVelocities[i],
                                force=1000)
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
            jointPoses = calculateInverseKinematics(armId, armEndEffectorId, pos,
                                                    threshold, maxIter)
    if useSimulation:
        for i in range(numJoints):
            p.setJointMotorControl2(bodyIndex=armId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=500,
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
