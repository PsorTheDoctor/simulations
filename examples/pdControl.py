import pybullet as p
import pybullet_data
from pdControllerStable import PDControllerStable
import time

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

pole = p.loadURDF('cartpole.urdf', [0,0,0], useMaximalCoordinates=False)

pd = PDControllerStable(p)

for i in range(p.getNumJoints(pole)):
    p.setJointMotorControl2(pole, i, p.POSITION_CONTROL, targetPosition=0, force=0)

timeStep = 0.01
desiredPosCart = 2
desiredVelCart = 0
kpCart = 1300
kdCart = 150
maxForceCart = 1000

desiredPosPole = 0
desiredVelPole = 0
kpPole = 1200
kdPole = 100
maxForcePole = 1000

p.loadPlugin('pdControlPlugin')
p.setGravity(0, 0, -9.81)

useRealTimeSim = False
p.setRealTimeSimulation(useRealTimeSim)

while p.isConnected():
    p.setTimeStep(timeStep)

    taus = pd.computePD(pole, [0, 1], [desiredPosCart, desiredPosPole],
                        [desiredVelCart, desiredVelPole], [kpCart, kpPole], [kdCart, kdPole],
                        [maxForceCart, maxForcePole], timeStep)
    for i in [0, 1]:
        p.setJointMotorControlMultiDof(pole, i, controlMode=p.TORQUE_CONTROL, force=[taus[i]])

    poleOrn = p.getLinkState(pole, linkIndex=1)[1]
    angle = p.getEulerFromQuaternion(poleOrn)

    print('Angle:', round(angle[1], 2))

    if not useRealTimeSim:
        p.stepSimulation()
        time.sleep(timeStep)
