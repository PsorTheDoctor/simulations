import pybullet as p
import pybullet_data

client = p.connect(p.SHARED_MEMORY)
if client < 0:
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
cube = p.loadURDF('cube.urdf', useFixedBase=True)

rollId = p.addUserDebugParameter('Roll', -1.5, 1.5, 0)
pitchId = p.addUserDebugParameter('Pitch', -1.5, 1.5, 0)
yawId = p.addUserDebugParameter('Yaw', -1.5, 1.5, 0)
xId = p.addUserDebugParameter('X', -1, 1, 0)
yId = p.addUserDebugParameter('Y', -1, 1, 0)
zId = p.addUserDebugParameter('Z', -1, 1, 0)

while True:
    roll = p.readUserDebugParameter(rollId)
    pitch = p.readUserDebugParameter(pitchId)
    yaw = p.readUserDebugParameter(yawId)
    x = p.readUserDebugParameter(xId)
    y = p.readUserDebugParameter(yId)
    z = p.readUserDebugParameter(zId)

    orn = p.getQuaternionFromEuler([roll, pitch, yaw])
    p.resetBasePositionAndOrientation(cube, [x, y, z], orn)
