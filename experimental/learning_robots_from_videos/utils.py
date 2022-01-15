import pybullet as p
import pybullet_data
import numpy as np
import cv2
import time
import random

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)

plane = p.loadURDF('plane.urdf')
table = p.loadURDF('table/table.urdf')
robotId = p.loadSDF('kuka_iiwa/kuka_with_gripper2.sdf')
robot = robotId[0]
numJoints = p.getNumJoints(robot)
p.resetBasePositionAndOrientation(robot, [0, 0, 0.7], [0,0,0,1])

robotPos = p.getBasePositionAndOrientation(robot)[0]
p.resetDebugVisualizerCamera(cameraDistance=1.2, cameraYaw=180, cameraPitch=-41,
                             cameraTargetPosition=robotPos)


def randomColor():
    color = np.random.rand(4)
    color[3] = 1
    return color


def createDataset(filename, numSamples):
    with open(filename, 'w') as f:
        f.truncate(0)  # clears a file!

    lastTime = time.time()
    i = 0
    while i < numSamples:
        currentTime = time.time()

        if currentTime - lastTime > 1:
            p.changeVisualShape(plane, -1, rgbaColor=randomColor())
            p.changeVisualShape(table, -1, rgbaColor=randomColor())

            color = randomColor()
            for joint in range(numJoints):
                p.changeVisualShape(robot, joint, rgbaColor=color)

            jointPositions = np.random.rand(14) * 3.5 * random.random()
            for joint in range(numJoints):
                p.resetJointState(robot, joint, jointPositions[joint])

            decimals = 4
            with open(filename, 'a') as f:
                line = 'id: {} joints: '.format(i)
                for joint in jointPositions:
                    line += str(round(joint, decimals)) + ' '
                f.write(line + '\n')

            # for joint in range(numJoints):
            #     pos = p.getJointState(robot, joint)[0]
            #     vel = p.getJointState(robot, joint)[1]
            #     reactionForces = p.getJointState(robot, joint)[2]
            #     torque = p.getJointState(robot, joint)[3]
            #
            #     decimals = 4
            #     with open('report.txt', 'a') as f:
            #         f.write('joint: {}\n'.format(joint))
            #         f.write('pos: {}\n'.format(round(pos, decimals)))
            #         f.write('vel: {}\n'.format(round(vel, decimals)))
            #         f.write('reactionForces: {}\n'.format(np.round_(reactionForces, decimals)))
            #         f.write('torque: {}\n'.format(round(torque, decimals)))
            #
            # with open(filename, 'a') as f:
            #     f.write('\n')

            img = p.getCameraImage(224, 224)[2]
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            cv2.imwrite('images2/{}.jpg'.format(i), gray)

            lastTime = currentTime
            i += 1

        p.stepSimulation()
        time.sleep(1. / 240.)
    p.disconnect()


createDataset('report2.txt', 10000)
