import pybullet as p
from time import sleep

p.connect(p.GUI)
p.setGravity(0, 0, -9.81)

ballCol = p.createCollisionShape(shapeType=p.GEOM_SPHERE, radius=0.1)
ballVis = p.createVisualShape(shapeType=p.GEOM_SPHERE, radius=0.1, rgbaColor=[1,0,0,1])
p.createMultiBody(baseCollisionShapeIndex=ballCol,
                  baseVisualShapeIndex=ballVis,
                  basePosition=[0, 0, 1])

for i in range(10000):
    p.stepSimulation()
    sleep(1./240.)

p.disconnect()
