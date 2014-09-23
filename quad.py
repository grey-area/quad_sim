import numpy as np

import quaternion

class Quad():
    def __init__(self, r, r_dot, omega, q, c, d, g, m, I): 
        # State
        self.r     = r
        self.r_dot = r_dot
        self.omega = omega
        self.q     = q

        # Parameters
        self.c = c
        self.d = d
        self.g = g
        self.m = m
        self.I = I

        self.inputs = np.zeros(4)

    # Simulate small time step
    def sim_step(self, dt):

        forces = np.copy(self.inputs) # TODO determine properly

        # Matrix for getting Fz, and the three moments, from the forces
        coeffs = np.array([[1,1,1,1],
                      [0,-self.d,0,self.d],
                      [-self.d,0,self.d,0],
                      [self.c,-self.c,self.c,-self.c]])
        M0 = np.dot(coeffs, forces)

        # Linear velocity, position update
        Fz = np.array([0,0,0,M0[0]])
        a = np.array([0,0,0,self.g]) + 1/self.m * quaternion.rotateVector(Fz, self.q) - 1/self.m * self.r_dot*0.6 # TODO proper friction coefficient, and that other term from the paper (omega cross velocity?)
        self.r_dot += dt * a
        self.r     += dt * self.r_dot

        # Bounce if you hit the floor
        if self.r[3]<0:
            self.r[3]=0
            self.r_dot[3]=-self.r_dot[3]
            self.r_dot *= 0.8

        # Angular vecolity, attitude quaternion update
        M = M0[1:]
        omega_dot = np.dot(np.linalg.inv(self.I), M - np.cross(self.omega, np.dot(self.I,self.omega)) )
        # Euler integration
        # TODO RK4? Geometric?
        self.omega += dt * omega_dot # TODO is this right?
        q_dot = quaternion.qmul(self.q, np.concatenate([[0],self.omega]))/2
        self.q += dt * q_dot # TODO is this right?
        self.q = quaternion.normalizedQuaternion(self.q)























