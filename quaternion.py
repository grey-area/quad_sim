import numpy as np

# Norm of a quaternion
def norm(q):
    return q[0]**2 + q[1]**2 + q[2]**2 + q[3]**2

# Returns normalized quaternion
def normalizedQuaternion(q):
    return q/norm(q)

# Quaternion multiplication
def qmul(q1, q2):
    return np.array([q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] - q1[3]*q2[3] ,
            q1[0]*q2[1] + q1[1]*q2[0] + q1[2]*q2[3] - q1[3]*q2[2] ,
            q1[0]*q2[2] - q1[1]*q2[3] + q1[2]*q2[0] + q1[3]*q2[1] ,
                     q1[0]*q2[3] + q1[1]*q2[2] - q1[2]*q2[1] + q1[3]*q2[0] ])

# Quaternion inverse
def qinv(q):
    q_inv = -q
    q_inv[0] = -q_inv[0]
    return q_inv

# Rotate vector using quaternion
def rotateVector(v, q):
    q_inv = qinv(q)
    return qmul(q, qmul(v, q_inv))

# Rotate vector using quaternion (inverse)
def rotateVectorI(v, q):
    q_inv = qinv(q)
    return qmul(q_inv, qmul(v, q))
