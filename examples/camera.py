import numpy as np
import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')
p.loadURDF('teddy_large.urdf', [0, 0, 1])
p.setGravity(0, 0, -9.81)

width = 128
height = 128

viewMat = [
    0.642787516117096, -0.4393851161003113, 0.6275069713592529, 0.0, 0.766044557094574,
    0.36868777871131897, -0.5265407562255859, 0.0, -0.0, 0.8191521167755127, 0.5735764503479004,
    0.0, 2.384185791015625e-07, 2.384185791015625e-07, -5.000000476837158, 1.0
]
projMat = [
    0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0,
    0.0, 0.0, -0.02000020071864128, 0.0
]
images = p.getCameraImage(width,
                          height,
                          viewMatrix=viewMat,
                          projectionMatrix=projMat,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)

np.reshape(images[2], (height, width, 4)) * 1. / 255.
# time.sleep(1)

images = p.getCameraImage(width,
                          height,
                          viewMatrix=viewMat,
                          projectionMatrix=projMat,
                          renderer=p.ER_TINY_RENDERER)

np.reshape(images[2], (height, width, 4)) * 1. / 255.

for i in range(10000):
    p.stepSimulation()
    camInfo = p.getDebugVisualizerCamera()
    print("View mat:", np.round(camInfo[2], 2))
    print("Proj mat:", np.round(camInfo[3], 2))
    time.sleep(1./240.)

p.disconnect()
