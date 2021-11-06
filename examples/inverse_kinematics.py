import pybullet as p
import pybullet_data
import math
from datetime import datetime

client = p.connect(p.SHARED_MEMORY)
if client < 0:
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF('plane.urdf', [0, 0, -0.3])
robotId = p.loadURDF('kuka_iiwa/model.urdf', [0, 0, 0])
p.resetBasePositionAndOrientation(robotId, [0, 0, 0], [0, 0, 0, 1])

numJoints = p.getNumJoints(robotId)
if numJoints != 7:
    exit()
robotEndEffectorIndex = numJoints - 1

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
    p.resetJointState(robotId, i, rp[i])

p.setGravity(0, 0, 0)
t = 0.
prevPose = [0, 0, 0]
prevPose1 = [0, 0, 0]
hasPrevPose = False
useNullSpace = True
useOrientation = True
useSimulation = True
useRealTimeSimulation = False
ikSolver = False
p.setRealTimeSimulation(useRealTimeSimulation)
trailDuration = 15

i = 0
while True:
    i += 1
    if useRealTimeSimulation:
        dt = datetime.now()
        t = (dt.second / 60.) * 2. * math.pi
    else:
        t = t + 0.01

    if useSimulation and not useRealTimeSimulation:
        p.stepSimulation()

    # Drawing a circle
    pos = [-0.4, 0.2 * math.cos(t), 0. + 0.2 * math.sin(t)]
    # End effector points down
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])

    if useNullSpace:
        if useOrientation:
            jointPoses = p.calculateInverseKinematics(robotId, robotEndEffectorIndex,
                                                      pos, orn, ll, ul, jr, rp)
        else:
            jointPoses = p.calculateInverseKinematics(robotId,
                                                      robotEndEffectorIndex,
                                                      pos,
                                                      lowerLimits=ll,
                                                      upperLimits=ul,
                                                      jointRanges=jr,
                                                      restPoses=rp)
    else:
        if useOrientation:
            jointPoses = p.calculateInverseKinematics(robotId,
                                                      robotEndEffectorIndex,
                                                      pos,
                                                      orn,
                                                      jointDamping=jd,
                                                      solver=ikSolver,
                                                      maxNumIterations=100,
                                                      residualThreshold=.01)
        else:
            jointPoses = p.calculateInverseKinematics(robotId,
                                                      robotEndEffectorIndex,
                                                      pos,
                                                      solver=ikSolver)
    if useSimulation:
        for i in range(numJoints):
            p.setJointMotorControl2(bodyIndex=robotId,
                                    jointIndex=i,
                                    controlMode=p.POSITION_CONTROL,
                                    targetPosition=jointPoses[i],
                                    targetVelocity=0,
                                    force=500,
                                    positionGain=0.03,
                                    velocityGain=1)
    else:
        # Reset joint state
        for i in range(numJoints):
            p.resetJointState(robotId, i, jointPoses[i])

    ls = p.getLinkState(robotId, robotEndEffectorIndex)
    if hasPrevPose:
        p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, trailDuration)
        p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
    prevPose = pos
    prevPose1 = ls[4]
    hasPrevPose = True
