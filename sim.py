import numpy as np
import math
import quad, control, quaternion


# Parameters
# TODO realistic parameters
c = 1                     #
d = 1                     #
g = -9.8                  # Gravity
m = 1                     # Copter mass
I = np.array([[1.,0.,0.], # Moment of inertia matrix
              [0.,1.,0.],
              [0.,0.,1.]]) 

# Initial state
r     = np.array([0, 0, 0, 7.0])  # Position
r_dot = np.array([0, 0, 0, 1.0])  # Velocity
omega     = (np.random.random(3)-0.5)*0.1 # Angular velocity
q = np.array([1.0, 0.0, 0.0, 0.0]) # Attitude quaternion

target_altitude = r[3]
target_yaw = 0.0
target_q = np.array([1.0, 0.0, 0.0, 0.0])
pitching  = 0
yawing    = 0
ascending = 0
copter = quad.Quad(r, r_dot, omega, q, c, d, g, m, I)





import pyglet
from pyglet.gl import *
from pyglet.window import key

width = 800
height = 800
window = pyglet.window.Window(width, height)
glClearColor(1,1,1,1)

# A glVertex transformed by the inertial-to-body-frame quaternion
def glVertex3fq(x,y,z,q):
    v = quaternion.rotateVector(np.array([0,x,y,z]), q)
    glVertex3f(v[1],v[2],v[3])

# Copture keyboard events
@window.event
def on_key_press(symbol, modifiers):
    global pitching, yawing, ascending
    if symbol == key.UP:
        pitching = 1
    elif symbol == key.DOWN:
        pitching = -1
    elif symbol == key.LEFT:
        yawing = 1
    elif symbol == key.RIGHT:
        yawing = -1
    elif symbol == ord('a'):
        ascending = 1
    elif symbol == ord('z'):
        ascending = -1
@window.event
def on_key_release(symbol, modifiers):
    global pitching, yawing, ascending
    if symbol == key.UP:
        pitching = 0
    elif symbol == key.DOWN:
        pitching = 0
    elif symbol == key.LEFT:
        yawing = 0
    elif symbol == key.RIGHT:
        yawing = 0
    elif symbol == ord('a'):
        ascending = 0
    elif symbol == ord('z'):
        ascending = 0

@window.event
def on_draw():
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    glLoadIdentity()

    glPushMatrix()
    # Camera position/rotation
    glTranslatef(0, 0, -10)
    glRotatef(90,0,0,1)
    glRotatef(70,0,1,0)
    glRotatef(20,0,0,1)

    glTranslatef(-copter.r[1],-copter.r[2],-copter.r[3])

    # Draw the floor
    tileSize = 4
    glPushMatrix()
    # Translate the floor in tileSize units so that we always have some floor below us
    glTranslatef(int(copter.r[1]/tileSize)*tileSize,int(copter.r[2]/tileSize)*tileSize,0)
    glColor3f(0.2,0.2,1.0)
    glLineWidth(1)
    glBegin(GL_LINES)
    for j in range(-140,140,10*tileSize):
        for i in range(-140,140,tileSize):
            glVertex3f(j,i,0)
            glVertex3f(j+10*tileSize,i,0)
            glVertex3f(i,j,0)
            glVertex3f(i,j+10*tileSize,0)
    glEnd()
    glPopMatrix()

    # Draw the shadow
    glPushMatrix()
    glTranslatef(r[1],r[2],0)
    glColor3f(0.6,0.6,0.6)
    glLineWidth(30)
    glBegin(GL_TRIANGLE_FAN)
    glVertex3f(0,0,0)
    for i in range(101):
        angle = -i*2*math.pi/100.0
        glVertex3f(math.sin(angle),math.cos(angle),0)
    glEnd()
    glPopMatrix()

    # Draw the 'ghost image' at the target altitude
    glPushMatrix()
    glTranslatef(r[1],r[2]+4,target_altitude)
    glColor3f(0.6,0.6,0.6)
    glLineWidth(4)
    glBegin(GL_LINES)
    glVertex3fq(-1,0,0,target_q)
    glVertex3fq(1,0,0,target_q)
    glVertex3fq(0,-1,0,target_q)
    glVertex3fq(0,1,0,target_q)
    glEnd()
    glPopMatrix()

    # Draw the copter
    glTranslatef(copter.r[1],copter.r[2],copter.r[3])
    # Draw the velocity vector
    glColor3f(0,0.5,0)
    glLineWidth(3)
    glBegin(GL_LINES)
    glVertex3f(0,0,0)
    glVertex3f(copter.r_dot[1]*0.3,copter.r_dot[2]*0.3,copter.r_dot[3]*0.3)
    glEnd()
    # Draw the angular velocity vector
    glColor3f(0,0,0.5)
    glLineWidth(3)
    glBegin(GL_LINES)
    glVertex3f(0,0,0)
    glVertex3f(copter.omega[0]*3,copter.omega[1]*3,copter.omega[2]*3)
    glEnd()
    # Draw the copter body
    glColor3f(0,0,0)
    glLineWidth(4)
    glBegin(GL_LINES)
    glVertex3fq(-1,0,0,copter.q)
    glVertex3fq(1,0,0,copter.q)
    glVertex3fq(0,-1,0,copter.q)
    glVertex3fq(0,1,0,copter.q)
    glEnd()
    # Draw the inputs for each rotor, with the front rotor a different colour
    glColor3f(1,0,0)
    glLineWidth(3)
    glBegin(GL_LINES)
    glVertex3fq(1,0,0,copter.q)
    glVertex3fq(1,0,copter.inputs[0],copter.q)
    glEnd()
    glColor3f(1,0.7,0)
    glBegin(GL_LINES)
    glVertex3fq(-1,0,0,copter.q)
    glVertex3fq(-1,0,copter.inputs[2],copter.q)
    glVertex3fq(0,-1,0,copter.q)
    glVertex3fq(0,-1,copter.inputs[1],copter.q)
    glVertex3fq(0,1,0,copter.q)
    glVertex3fq(0,1,copter.inputs[3],copter.q)
    glEnd()
    
    glPopMatrix()

@window.event
def on_resize(width, height):
    glViewport(0, 0, width, height)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(65, width / float(height), .1, 300)
    glMatrixMode(GL_MODELVIEW)
    return pyglet.event.EVENT_HANDLED

# If the graphics are screwy, comment out this whole block
glEnable(GL_DEPTH_TEST)
glDepthFunc(GL_LESS)
glEnable(GL_FOG)
glFogi(GL_FOG_MODE, GL_LINEAR)
glFogf(GL_FOG_START, 80.0)
glFogf(GL_FOG_END, 120.0)
glHint(GL_FOG_HINT, GL_DONT_CARE)
fogColour = [1,1,1,1]
#glFogfv(GL_FOG_COLOR, [GLfloat(x) for x in [1.0,1.0,1.0,1.0]])
glFogfv(GL_FOG_COLOR, (GLfloat * len(fogColour))(*fogColour))
glFogf (GL_FOG_DENSITY, 0.95)
glClearColor(1.0, 1.0, 1.0, 1.0)


def update(dt):
    global target_altitude, target_yaw, target_q
    target_altitude += ascending * dt * 3
    target_yaw += yawing * dt * 0.4
    target_q = np.array([math.cos(target_yaw/2),0.,0.,math.sin(target_yaw/2)])
    pitching_angle = 15*math.pi/180.0 * pitching
    pitching_q = np.array([math.cos(pitching_angle/2),0,math.sin(pitching_angle/2),0])
    target_q = quaternion.qmul(pitching_q, target_q)

    copter.inputs = control.inputs(copter.q, copter.r[3], target_q, target_altitude)
    copter.sim_step(dt)

pyglet.clock.schedule(update)
pyglet.app.run()
