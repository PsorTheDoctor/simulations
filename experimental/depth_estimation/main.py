import pybullet as p
import pybullet_data
import time
import matplotlib.pyplot as plt
from experimental.depth_estimation.utils import *

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
plane = p.loadURDF('plane.urdf')
p.changeDynamics(plane, -1, lateralFriction=60)

husky = p.loadURDF('husky/husky.urdf')
wall = createWall(dist=5, width=2)

camJoint = 7
camInfo = p.getDebugVisualizerCamera()

vel = 10
wheels = [2, 3, 4, 5]

bumped = False
widths = []
distances = []
lastTime = time.time()

while not bumped:
    img = getOnboardCamera(husky, camJoint, camInfo)

    currTime = time.time()
    if currTime - lastTime > 1:
        segmented = img[4]
        wallWidth = measureWidth(segmented)
        dist = measureDist(husky, wall)
        widths.append(wallWidth)
        distances.append(dist)

        bumped = wallWidth == 0 or wallWidth == 400
        lastTime = currTime

    for i in range(len(wheels)):
        p.setJointMotorControl2(husky, wheels[i], p.VELOCITY_CONTROL,
                                targetVelocity=vel, force=30)
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()

plt.plot(widths)
plt.legend(['widths'])
plt.show()

plt.plot(distances)
plt.legend(['distances'])
plt.show()
