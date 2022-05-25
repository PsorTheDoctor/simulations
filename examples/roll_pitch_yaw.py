import pybullet as p
import pybullet_data

client = p.connect(p.SHARED_MEMORY)
if client < 0:
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
# cube = p.loadURDF('cube.urdf', useFixedBase=True)


def loadSTL(filename, basePosition=(0, 0, 0), baseOrientation=(0, 0, 0, 1)):

    col_shape_id = p.createCollisionShape(
        shapeType=p.GEOM_MESH,
        fileName=filename,
        flags=p.URDF_INITIALIZE_SAT_FEATURES #|p.GEOM_FORCE_CONCAVE_TRIMESH should only be used with fixed (mass=0) objects!
    )
    viz_shape_id = p.createVisualShape(
        shapeType=p.GEOM_MESH,
        fileName=filename,
    )
    body_id = p.createMultiBody(
        baseMass=1,
        baseCollisionShapeIndex=col_shape_id,
        baseVisualShapeIndex=viz_shape_id,
        basePosition=basePosition,
        baseOrientation=baseOrientation
    )
    return body_id


bunny = loadSTL('../data/bunny2.stl')

rollId = p.addUserDebugParameter('Roll', -1.5, 1.5, 0)
pitchId = p.addUserDebugParameter('Pitch', -3.14, 3.14, 0)
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
    p.resetBasePositionAndOrientation(bunny, [x, y, z], orn)
