import pybullet as p
import pybullet_data
import numpy as np
import time


def contact():
    tip_z_pos = p.getLinkState(hopperId, tipLink)[0][2]
    if tip_z_pos < 0.012:
        return True
    else:
        return False


def getVelocity():
    base_vel = np.array(p.getBaseVelocity(hopperId)[0])
    link_vel = np.array(p.getLinkState(hopperId, 0, 1)[6])

    hopper_vel = (base_vel + link_vel) / 2
    return hopper_vel


def getTargetLegDisplacement():
    vel = getVelocity()[0:2]
    vel_error = vel - targetVel
    vel_gain = 0.027

    neural_point = (vel * stance_duration) / 2.0
    disp = neural_point + vel_gain * vel_error
    disp = np.append(disp, -np.sqrt(getLegLength()**2 - disp[0]**2 - disp[1]**2))
    return disp


def getLegLength():
    return 0.15 - p.getJointState(hopperId, pneumatic_joint)[0]


def transform_H_to_B(vec):
    HB_matrix_row_form = p.getMatrixFromQuaternion(p.getBasePositionAndOrientation(hopperId)[1])
    HB_matrix = np.zeros((4, 4))
    HB_matrix[3, 3] = 1
    HB_matrix[0, 0:3] = HB_matrix_row_form[0:3]
    HB_matrix[1, 0:3] = HB_matrix_row_form[3:6]
    HB_matrix[2, 0:3] = HB_matrix_row_form[6:9]

    HB_matrix = np.matrix(HB_matrix)
    BH_matrix = np.linalg.inv(HB_matrix)
    return BH_matrix * vec


# k_flight = 1200  # leg spring constant during flight phase
# k_stance = 2700  # leg spring constant during stance phase
k = 1000
state = 0
legForce = 0
tipLink = 6

outer_hip_joint = 2
inner_hip_joint = 5
pneumatic_joint = 6

hip_joint_kp = 10
hip_joint_kd = 0.5

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

targetVelX = p.addUserDebugParameter('Target velocity X', -0.3, 0.3, 0)
targetVelY = p.addUserDebugParameter('Target velocity Y', -0.3, 0.3, 0)
actuatorSoftness = p.addUserDebugParameter('Actuator softness', 0, 2000, 1000)

planeId = p.loadURDF('plane.urdf')
p.changeDynamics(planeId, -1, lateralFriction=60)
p.resetDebugVisualizerCamera(cameraDistance=1.0, cameraYaw=135, cameraPitch=-30,
                             cameraTargetPosition=[0, 0, 0])

hopperId = p.loadURDF('../data/hopper.urdf', [0, 0, 0.2], [0, 0.001, 0, 1])

p.setJointMotorControl2(hopperId, pneumatic_joint, p.VELOCITY_CONTROL, force=0)

p.setGravity(0, 0, -9.81)
curtime = 0
dt = 1. / 240.

prev_orientation = np.array([0, 0, 0])
count = 0
stance_made = False
stance_duration = 0.17
targetVel = np.array([0.3, 0.3])

while True:
    count = count + 1
    curtime = curtime + dt
    pos = p.getJointState(hopperId, pneumatic_joint)[0]

    if contact():  # stance phase
        if not stance_made:
            stance_made = True
            stance_duration = 0
        stance_duration = stance_duration + dt
        base_orientation = p.getLinkState(hopperId, 1)[1]
        base_orientation_euler = np.array(p.getEulerFromQuaternion(base_orientation))

        orientation_change = base_orientation_euler - prev_orientation
        orientation_vel = orientation_change / dt

        outer_hip_joint_target_vel = -hip_joint_kp * base_orientation_euler[0] - hip_joint_kd * orientation_vel[0]
        inner_hip_joint_target_vel = -hip_joint_kp * base_orientation_euler[1] - hip_joint_kd * orientation_vel[1]

        p.setJointMotorControl2(hopperId, outer_hip_joint, p.VELOCITY_CONTROL,
                                targetVelocity=outer_hip_joint_target_vel)
        p.setJointMotorControl2(hopperId, inner_hip_joint, p.VELOCITY_CONTROL,
                                targetVelocity=inner_hip_joint_target_vel)

        prev_orientation = base_orientation_euler
        legForce = -k * pos - 20

    else:  # flight phase
        stance_made = False
        legForce = -k * pos
        targetLegDisplacement_H = getTargetLegDisplacement()
        targetLegDisplacement_H = np.append(targetLegDisplacement_H, 1)
        targetLegDisplacement_H = np.matrix(targetLegDisplacement_H)
        targetLegDisplacement_H = targetLegDisplacement_H.T

        targetLegDisplacement_B = transform_H_to_B(targetLegDisplacement_H)

        x_disp_B = targetLegDisplacement_B[0, 0]
        y_disp_B = targetLegDisplacement_B[1, 0]

        d = getLegLength()
        theta_inner = np.arcsin(x_disp_B / d)
        theta_outer = np.arcsin(y_disp_B / (-d * np.cos(theta_inner)))

        p.setJointMotorControl2(hopperId, outer_hip_joint, p.POSITION_CONTROL,
                                targetPosition=theta_outer)
        p.setJointMotorControl2(hopperId, inner_hip_joint, p.POSITION_CONTROL,
                                targetPosition=theta_inner)

    p.setJointMotorControl2(hopperId, pneumatic_joint, p.TORQUE_CONTROL, force=legForce)

    targetVel[0] = p.readUserDebugParameter(targetVelX)
    targetVel[1] = p.readUserDebugParameter(targetVelY)
    k = p.readUserDebugParameter(actuatorSoftness)

    p.stepSimulation()
    time.sleep(dt)
