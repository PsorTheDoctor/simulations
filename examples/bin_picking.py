import pybullet as p
import pybullet_data
import time
import random
import math
import matplotlib.pyplot as plt

client = p.connect(p.SHARED_MEMORY)
if client < 0:
    p.connect(p.GUI)

p.resetDebugVisualizerCamera(1.3, 180, -41, [0.52, -0.2, -0.33])

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.loadURDF('plane.urdf', [0, 0, -1])

p.loadURDF('table/table.urdf', 0.5, 0.0, -0.82, 0.0, 0.0, 0.0, 1.0)

xpos = 0.5 + 0.2 * random.random()
ypos = 0.25 * random.random()
ang = math.pi * random.random()
orn = p.getQuaternionFromEuler([0, 0, ang])
p.loadURDF('block.urdf', xpos, ypos, -0.1, orn[0], orn[1], orn[2], orn[3])

p.setGravity(0, 0, -9.81)

objects = p.loadSDF('kuka_iiwa/kuka_with_gripper2.sdf')
robot = objects[0]
p.resetBasePositionAndOrientation(robot, [-0.1, 0.0, 0.07], [0.0, 0.0, 0.0, 1.0])
jointPositions = [
    0.006418, 0.413184, -0.011401, -1.589317, 0.005379, 1.137684, -0.006539, 0.000048,
    -0.299912, 0.000000, -0.000043, 0.299960, 0.000000, -0.000200
]

p.loadURDF('tray/tray.urdf', 0.640000, 0.075000, -0.19, 0.0, 0.0, 1.0, 0.0)

viewMat = [
    -0.5120397806167603, 0.7171027660369873, -0.47284144163131714, 0.0, -0.8589617609977722,
    -0.42747554183006287, 0.28186774253845215, 0.0, 0.0, 0.5504802465438843,
    0.8348482847213745, 0.0, 0.1925382763147354, -0.24935829639434814, -0.4401884973049164, 1.0
]
projMat = [
    0.75, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0,
    -0.02000020071864128, 0.0
]
width = 160
height = 120

img_arr = p.getCameraImage(width=width,
                           height=height,
                           viewMatrix=viewMat,
                           projectionMatrix=projMat)
rgb = img_arr[2]
depth = img_arr[3]

btn = p.addUserDebugParameter('Show point cloud', 1, 0, startValue=0)

for i in range(10000):
    if p.readUserDebugParameter(btn) % 2 == 1:
        plt.figure()
        plt.imshow(depth * 255.0, cmap='viridis')
        plt.show()

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
