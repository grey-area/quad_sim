import numpy as np
import math
import quaternion

# Given the attitude quaternion and the position vector, set the inputs of the motors
# TODO. This is a placeholder controller. Imlement PID? LPQ?
# all temporary, using pitch, roll, yaw
def inputs(actual_q, altitude, target_q, target_altitude):
    inpt = np.ones(4)*9.8/4 # TODO what range should these be in?

    q = quaternion.qmul(quaternion.qinv(target_q), actual_q)

    roll = math.atan2(2*(q[2]*q[3] + q[0]*q[1]), q[0]*q[0] - q[1]*q[1] - q[2]*q[2] + q[3]*q[3])
    yaw = math.atan2(2*(q[1]*q[2] + q[0]*q[3]), q[0]*q[0] + q[1]*q[1] - q[2]*q[2] - q[3]*q[3])
    pitch = math.asin(-2*(q[1]*q[3] - q[0]*q[2]))

    inpt[0] += 0.3*(target_altitude-altitude)
    inpt[1] += 0.3*(target_altitude-altitude)
    inpt[2] += 0.3*(target_altitude-altitude)
    inpt[3] += 0.3*(target_altitude-altitude)
    inpt = [max(0.3,x) for x in inpt]

    inpt[0] -= 0.3*yaw
    inpt[2] -= 0.3*yaw
    inpt[1] += 0.3*yaw
    inpt[3] += 0.3*yaw
    inpt = [max(0,x) for x in inpt]

    inpt[0] += 0.2*pitch
    inpt[2] -= 0.2*pitch
    inpt[1] += 0.2*roll
    inpt[3] -= 0.2*roll
    inpt = [max(0,x) for x in inpt]

    return inpt
