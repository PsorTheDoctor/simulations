import cv2
import pybullet as p
import time
import pybullet_data

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.loadURDF('plane.urdf')
p.loadURDF('teddy_large.urdf', [0, 0, 1])
p.setGravity(0, 0, -9.81)

record = True
width = 512
height = 512

fourcc = cv2.VideoWriter_fourcc(*'XVID')
writer = cv2.VideoWriter('result.mp4', fourcc, 24, (width, height))

for i in range(100):
    if record:
        images = p.getCameraImage(width, height, renderer=p.ER_TINY_RENDERER)
        rgb = images[2]
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        writer.write(bgr)
        print(i)

    p.stepSimulation()
    time.sleep(1./240.)

writer.release()
p.disconnect()
