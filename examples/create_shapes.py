import pybullet as p
from time import sleep
import pybullet_data

client = p.connect(p.SHARED_MEMORY)
if client < 0:
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setPhysicsEngineParameter(numSolverIterations=10)
p.setTimeStep(1. / 120.)
logId = p.startStateLogging(p.STATE_LOGGING_PROFILE_TIMINGS, 'visualShapeBench.json')
# useMaximalCoordinates is much faster than default
p.loadURDF('plane100.urdf', useMaximalCoordinates=True)
# Disable rendering during creation
p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, False)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
# Disable tinyrenderer, software (CPU) renderer
p.configureDebugVisualizer(p.COV_ENABLE_TINY_RENDERER, False)

shift = [0, -0.02, 0]
meshScale = [0.1, 0.1, 0.1]

visualShapeId = p.createVisualShape(shapeType=p.GEOM_MESH,
                                    fileName='duck.obj',
                                    rgbaColor=[1, 1, 1, 1],
                                    specularColor=[.4, .4, 0],
                                    visualFramePosition=shift,
                                    meshScale=meshScale)
collisionShapeId = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                          fileName='duck_vhacd.obj',
                                          meshScale=meshScale)
rangeX = 2
rangeY = 2
for i in range(rangeX):
    for j in range(rangeY):
        p.createMultiBody(baseMass=1,
                          baseInertialFramePosition=[0, 0, 0],
                          baseCollisionShapeIndex=collisionShapeId,
                          baseVisualShapeIndex=visualShapeId,
                          basePosition=[((-rangeX / 2) + i) * meshScale[0] * 2,
                                        (-rangeY / 2 + j) * meshScale[1] * 2, 1],
                          useMaximalCoordinates=True)

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, True)
p.stopStateLogging(logId)
p.setGravity(0, 0, -10)
p.setRealTimeSimulation(True)

while True:
    sleep(1./240.)
