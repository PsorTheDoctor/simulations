import numpy as np


class PDControllerStable(object):
    def __init__(self, p):
        self.p = p

    def computePD(self, bodyId, jointIndices, desiredPositions, desiredVelocities, kps, kds,
                  maxForces, timeStep):
        numJoints = self.p.getNumJoints(bodyId)
        jointStates = self.p.getJointStates(bodyId, jointIndices)
        q1 = []
        qdot1 = []
        zeroAccelerations = []

        for i in range(numJoints):
            q1.append(jointStates[i][0])
            qdot1.append(jointStates[i][1])
            zeroAccelerations.append(0)

        q = np.array(q1)
        qdot = np.array(qdot1)
        qdes = np.array(desiredPositions)
        qdotdes = np.array(desiredVelocities)

        qError = qdes - q
        qdotError = qdotdes - qdot

        Kp = np.diagflat(kps)
        Kd = np.diagflat(kds)

        p_term = Kp.dot(qError - qdot * timeStep)
        d_term = Kd.dot(qdotError)

        # Compute inertia matrix
        M = self.p.calculateMassMatrix(bodyId, q1)
        M = np.array(M)
        # Compute Coriolis ans external (gravitational) terms
        G = self.p.calculateInverseDynamics(bodyId, q1, qdot1, zeroAccelerations)
        G = np.array(G)
        # Obtain estimated generalized accelerations,
        # considering Coriolis, Gravitational forces and stable PD actions
        qddot = np.linalg.solve(a=(M + Kd * timeStep),
                                b=(-G + p_term + d_term))
        # Compute control generalized forces
        tau = p_term + d_term - (Kd.dot(qddot) * timeStep)
        # Clip generalized forces to actuator limits
        maxF = np.array(maxForces)
        generalizedForces = np.clip(tau, -maxF, maxF)
        return generalizedForces
