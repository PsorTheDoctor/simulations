import pybullet as p
import pybullet_data
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF('plane.urdf')
p.setGravity(0, 0, -9.81)
p.changeDynamics(plane, -1, lateralFriction=10)

quadruped = p.loadURDF('laikago/laikago_toes.urdf', [0, 0, .5], [0, 0.5, 0.5, 0])

for i in range(p.getNumJoints(quadruped)):
    print(p.getJointInfo(quadruped, i))

upper_legs  = [1, 5, 9, 13]
lower_legs =  [2, 6, 10, 14]
force = 100


def tapping():
    p.setJointMotorControl2(quadruped, 2, p.POSITION_CONTROL, -0.5, force=force)
    p.setJointMotorControl2(quadruped, 14, p.POSITION_CONTROL, -0.5, force=force)
    p.stepSimulation()

    if abs(p.getJointState(quadruped, 2)[0] + 0.5) < 0.1:
        p.setJointMotorControl2(quadruped, 2, p.POSITION_CONTROL, 0, force=force)
        p.setJointMotorControl2(quadruped, 14, p.POSITION_CONTROL, 0, force=force)
        p.stepSimulation()

    if abs(p.getJointState(quadruped, 2)[0]) < 0.1:
        p.setJointMotorControl2(quadruped, 6, p.POSITION_CONTROL, -0.5, force=force)
        p.setJointMotorControl2(quadruped, 10, p.POSITION_CONTROL, -0.5, force=force)
        p.stepSimulation()

    if abs(p.getJointState(quadruped, 6)[0] + 0.5) < 0.1:
        p.setJointMotorControl2(quadruped, 6, p.POSITION_CONTROL, 0, force=force)
        p.setJointMotorControl2(quadruped, 10, p.POSITION_CONTROL, 0, force=force)
        p.stepSimulation()


while True:
    # tapping()
    time.sleep(1. / 240.)
