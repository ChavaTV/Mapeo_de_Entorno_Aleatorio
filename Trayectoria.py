import numpy as np
import time
import math as m
import random
import matplotlib.pyplot as plt
import os
import scipy.interpolate as spi
import sim as vrep # access all the VREP elements
from skimage.draw import line

def q2R(x,y,z,w):
    R = np.zeros((3,3))
    R[0,0] = 1-2*(y**2+z**2)
    R[0,1] = 2*(x*y-z*w)
    R[0,2] = 2*(x*z+y*w)
    R[1,0] = 2*(x*y+z*w)
    R[1,1] = 1-2*(x**2+z**2)
    R[1,2] = 2*(y*z-x*w)
    R[2,0] = 2*(x*z-y*w)
    R[2,1] = 2*(y*z+x*w)
    R[2,2] = 1/2*(x**2+y**2)
    return R

def angdiff(t1, t2):
    """
    Compute the angle difference, t2-t1, restricting the result to the [-pi,pi] range
    """
    # The angle magnitude comes from the dot product of two vectors
    angmag = m.acos(m.cos(t1)*m.cos(t2)+m.sin(t1)*m.sin(t2))
    # The direction of rotation comes from the sign of the cross product of two vectors
    angdir = m.cos(t1)*m.sin(t2)-m.sin(t1)*m.cos(t2)
    return m.copysign(angmag, angdir)

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',-1,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")

# Getting handles for the motors and robot
err, motorL = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_blocking)
err, robot = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx', vrep.simx_opmode_blocking)

# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(1,17):
    err, s = vrep.simxGetObjectHandle(clientID, 'Pioneer_p3dx_ultrasonicSensor'+str(i), vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)

ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

Kv = 0.2
Kh = 0.8
xd = 3
yd = 3
hd = 0
r = 0.1
L = 0.2
errp = 10

if os.path.exists('map.txt'):
    print('Map found. Loading...')
    occgrid = np.loadtxt('map.txt')
    tocc = 1.0*(occgrid > 0.5)
    occgrid[occgrid > 0.5] = 0
else:
    print('Creating new map')
    occgrid = 0.5*np.ones((100,100))
    tocc = np.zeros((100,100))
t = time.time()

initt = t
niter = 0
xarr = np.array([1,-1,-1,1,1])
yarr = np.array([1,1,-1,-1,1])
print(xarr)
print(yarr)
tarr = np.linspace(0, 120, xarr.shape[0])
    
pcix = spi.PchipInterpolator(tarr, xarr)
pciy = spi.PchipInterpolator(tarr, yarr)

while time.time()-t < 65:
        
    tiempoS = time.time()-t
    errp = 10
    
    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)
    errp = m.sqrt((pcix(tiempoS)-carpos[0])**2 + (pciy(tiempoS)-carpos[1])**2)
    angd = m.atan2(pciy(tiempoS)-carpos[1], pcix(tiempoS)-carpos[0])
    errh = angdiff(carrot[2], angd)
    v = Kv*errp
    omega = Kh*errh
    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)
    vrep.simxAddStatusbarMessage(clientID, 'Following path', vrep.simx_opmode_oneshot)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    
    #ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
    
    xw = carpos[0]
    yw = carpos[1]
    xr = 50 + m.ceil(xw/0.1)
    yr = 50 - m.floor(yw/0.1)
    if xr >= 100:
        xr = 100
    if yr >= 100:
        yr = 100
    occgrid[yr-1, xr-1] = 0

    #ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)

    uread = []
    ustate = []
    upt = []
    for i in range(9):
       err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
       ret, objpos = vrep.simxGetObjectPosition(clientID, detectedObj, -1, vrep.simx_opmode_blocking)
       uread.append(np.linalg.norm(point))
       upt.append(point)
       ustate.append(state)
       ret, srot = vrep.simxGetObjectQuaternion(clientID, usensor[i], -1, vrep.simx_opmode_blocking)
       ret, spos = vrep.simxGetObjectPosition(clientID, usensor[i], -1, vrep.simx_opmode_blocking)
       R = q2R(srot[0], srot[1], srot[2], srot[3])
       spos = np.array(spos).reshape((3,1))
       if i % 2 != 0:
           continue
       if state == True:
           opos = np.array(point).reshape((3,1))
    
           pobs = np.matmul(R, opos) + spos
           xs = pobs[0]
           ys = pobs[1]
           xo = 50 + m.ceil(xs/0.1)
           yo = 50 - m.floor(ys/0.1)
           if xo >= 100:
               xo = 100
           if yo >= 100:
               yo = 100
            
           #rows, cols = line(yr-1, xr-1, yo-1, xo-1)
           #occgrid[rows, cols] = 0
           #tocc[yo-1, xo-1] = 1
# =============================================================================
#          En esta parte agregamos el wallfollower
#          Para seguir lo más que se pueda el contorno
#          durante los 5s que le damos a la rutina
# =============================================================================
           TimeStartWhile = time.time()
           while time.time()- TimeStartWhile < 25:
                tstart = time.time()
                smeasure = []
                statev = []
                r = 0.5*0.195
                for i in range(16):
                    err = 1
                    while err:
                        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
                    smeasure.append(np.linalg.norm(point))
                    statev.append(state)
                
                #if (statev[0] and smeasure[0] < 0.8) or (statev[1] and smeasure[1] < 0.8):
                if statev[4] or statev[5]:
                    v = 0.0
                    omega = 0.3
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Avoiding front wall', vrep.simx_opmode_oneshot)
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    #break
                elif statev[2] or statev[3]:
                    v = 0.0
                    omega = 0.3
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ur, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ul, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Avoiding front wall', vrep.simx_opmode_oneshot)
                    opos = np.array(point).reshape((3,1))
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    #break
                elif (statev[15] or statev[0] or statev[1]):
                #elif i == 2 and i==3:
                    D = 0.6
                    #print('D: {}'.format(0.5*np.abs(smeasure[2]+smeasure[3])))
                    d = 0.5*(smeasure[0]+smeasure[1]+smeasure[15])
                    v = 0.5
                    Kh = 0.8
                    omega = Kh*(D-d)
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Wall following {}'.format(D-d), vrep.simx_opmode_oneshot)
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    break
                elif (statev[7] or statev[8] or statev[6]):
                #elif i == 2 and i==3:
                    D = 0.6
                    #print('D: {}'.format(0.5*np.abs(smeasure[2]+smeasure[3])))
                    d = 0.5*(smeasure[6]+smeasure[7]+smeasure[8])
                    v = 0.5
                    Kh = 0.8
                    omega = Kh*(D-d)
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ur, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ul, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Wall following {}'.format(D-d), vrep.simx_opmode_oneshot)
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    break
                elif(statev[14] or statev[13] or statev[12] or statev[11] or statev[10] or statev[9]):
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    break
                else:
                    v = 0.2
                    omega = -0.5
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Searching for wall', vrep.simx_opmode_oneshot)
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    break
                time.sleep(0.001)
                #iter = iter + 1
                #print(time.time()-tstart)
                #print('sale del while')
                
                
#==============================================================================
       else:
           opos = np.array([0,0,1]).reshape((3,1))
           
           pobs = np.matmul(R, opos) + spos
           xs = pobs[0]
           ys = pobs[1]
           xo = 50 + m.ceil(xs/0.1)
           yo = 50 - m.floor(ys/0.1)
           if xo >= 100:
               xo = 100
           if yo >= 100:
               yo = 100
           rows, cols = line(yr-1, xr-1, yo-1, xo-1)
           occgrid[rows, cols] = 0

    niter = niter + 1

#print(lgains)
#print(rgains)
# =============================================================================
# Aquí entraría el modo de exploración
# =============================================================================
print('Iniciando modo exploracion')
Kv = 0.1
Kh = 0.5
Exploracion = np.random.randint(2,5,1)
xarr = xarr * Exploracion[0]
yarr = yarr * Exploracion[0]
print(xarr)
print(yarr)
tarr = np.linspace(0, 180, xarr.shape[0])
    
pcix = spi.PchipInterpolator(tarr, xarr)
pciy = spi.PchipInterpolator(tarr, yarr)

while time.time()-t < 120:
        
    tiempoS = time.time()-t
    errp = 10
    
    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)
    errp = m.sqrt((pcix(tiempoS)-carpos[0])**2 + (pciy(tiempoS)-carpos[1])**2)
    angd = m.atan2(pciy(tiempoS)-carpos[1], pcix(tiempoS)-carpos[0])
    errh = angdiff(carrot[2], angd)
    v = Kv*errp
    omega = Kh*errh
    ul = v/r - L*omega/(2*r)
    ur = v/r + L*omega/(2*r)
    vrep.simxAddStatusbarMessage(clientID, 'Following path', vrep.simx_opmode_oneshot)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
    
    #ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
    
    xw = carpos[0]
    yw = carpos[1]
    xr = 50 + m.ceil(xw/0.1)
    yr = 50 - m.floor(yw/0.1)
    if xr >= 100:
        xr = 100
    if yr >= 100:
        yr = 100
    occgrid[yr-1, xr-1] = 0

    #ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)

    uread = []
    ustate = []
    upt = []
    for i in range(9):
       err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
       ret, objpos = vrep.simxGetObjectPosition(clientID, detectedObj, -1, vrep.simx_opmode_blocking)
       uread.append(np.linalg.norm(point))
       upt.append(point)
       ustate.append(state)
       ret, srot = vrep.simxGetObjectQuaternion(clientID, usensor[i], -1, vrep.simx_opmode_blocking)
       ret, spos = vrep.simxGetObjectPosition(clientID, usensor[i], -1, vrep.simx_opmode_blocking)
       R = q2R(srot[0], srot[1], srot[2], srot[3])
       spos = np.array(spos).reshape((3,1))
       if i % 2 != 0:
           continue
       if state == True:
           opos = np.array(point).reshape((3,1))
    
           pobs = np.matmul(R, opos) + spos
           xs = pobs[0]
           ys = pobs[1]
           xo = 50 + m.ceil(xs/0.1)
           yo = 50 - m.floor(ys/0.1)
           if xo >= 100:
               xo = 100
           if yo >= 100:
               yo = 100
            
           #rows, cols = line(yr-1, xr-1, yo-1, xo-1)
           #occgrid[rows, cols] = 0
           #tocc[yo-1, xo-1] = 1
# =============================================================================
#          En esta parte agregamos el wallfollower
#          Para seguir lo más que se pueda el contorno
#          durante los 5s que le damos a la rutina
# =============================================================================
           TimeStartWhile2 = time.time()
           while time.time()- TimeStartWhile2 < 35:
                tstart = time.time()
                smeasure = []
                statev = []
                r = 0.5*0.195
                for i in range(9):
                    err = 1
                    while err:
                        err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_buffer)
                    smeasure.append(np.linalg.norm(point))
                    statev.append(state)
                
                #if (statev[0] and smeasure[0] < 0.8) or (statev[1] and smeasure[1] < 0.8):
                if statev[4] or statev[5]:
                    v = 0.0
                    omega = 0.3
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Avoiding front wall', vrep.simx_opmode_oneshot)
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    break
                elif statev[2] or statev[3]:
                    v = 0.0
                    omega = 0.3
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ur, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ul, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Avoiding front wall', vrep.simx_opmode_oneshot)
                    opos = np.array(point).reshape((3,1))
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    break
                elif (statev[0] or statev[1]):
                #elif i == 2 and i==3:
                    D = 0.6
                    #print('D: {}'.format(0.5*np.abs(smeasure[2]+smeasure[3])))
                    d = 0.5*(smeasure[0]+smeasure[1])
                    v = 0.5
                    Kh = 0.8
                    omega = Kh*(D-d)
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Wall following {}'.format(D-d), vrep.simx_opmode_oneshot)
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    break
                elif (statev[7] or statev[8] or statev[6]):
                #elif i == 2 and i==3:
                    D = 0.6
                    #print('D: {}'.format(0.5*np.abs(smeasure[2]+smeasure[3])))
                    d = 0.5*(smeasure[6]+smeasure[7]+smeasure[8])
                    v = 0.5
                    Kh = 0.8
                    omega = Kh*(D-d)
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ur, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ul, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Wall following {}'.format(D-d), vrep.simx_opmode_oneshot)
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    break
# =============================================================================
#                 elif(statev[14] or statev[13] or statev[12] or statev[11] or statev[10] or statev[9]):
#                     rows, cols = line(yr-1, xr-1, yo-1, xo-1)
#                     occgrid[rows, cols] = 0
#                     tocc[yo-1, xo-1] = 1
#                     break
# =============================================================================
                else:
                    v = 0.2
                    omega = -0.5
                    ul = v/r - L*omega/(2*r)
                    ur = v/r + L*omega/(2*r)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_streaming)
                    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_streaming)
                    vrep.simxAddStatusbarMessage(clientID, 'Searching for wall', vrep.simx_opmode_oneshot)
                    rows, cols = line(yr-1, xr-1, yo-1, xo-1)
                    occgrid[rows, cols] = 0
                    tocc[yo-1, xo-1] = 1
                    break
                time.sleep(0.001)
                #iter = iter + 1
                #print(time.time()-tstart)
                #print('sale del while')
                
                
#==============================================================================
       else:
           opos = np.array([0,0,1]).reshape((3,1))
           
           pobs = np.matmul(R, opos) + spos
           xs = pobs[0]
           ys = pobs[1]
           xo = 50 + m.ceil(xs/0.1)
           yo = 50 - m.floor(ys/0.1)
           if xo >= 100:
               xo = 100
           if yo >= 100:
               yo = 100
           rows, cols = line(yr-1, xr-1, yo-1, xo-1)
           occgrid[rows, cols] = 0

    niter = niter + 1


# =============================================================================
#Aquí termina el modo exploración
# =============================================================================
finalt = time.time()
#print('Avg time per iteration ', (finalt-initt)/niter)

print(tocc)
plt.imshow(tocc+occgrid)
plt.show()
np.savetxt('map.txt', tocc+occgrid)
errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0.0, vrep.simx_opmode_streaming)
errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0.0, vrep.simx_opmode_streaming)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)