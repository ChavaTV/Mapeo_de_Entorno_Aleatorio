"""
Alumno: Torres Villegas Antonio Salvador 
Proyecto C/C++ programming
Profesor: Manuel Alejandro Solis Arrazola
"""
import numpy as np
import time
import math as m
import random
import sys
import os
import matplotlib.pyplot as plt
import sim as vrep # access all the sim elements
from skimage.draw import line

#UTILICE UN ARCHIVO SIM QUE ES QUIEN REALIZA LA CONEXION ENTRE EL SIMULADOR Y EL CODIGO

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

vrep.simxFinish(-1) # just in case, close all opened connections
clientID=vrep.simxStart('127.0.0.1',-1,True,True,5000,5) # start a connection
if clientID!=-1:
	print ('Connected to remote API server')
else:
	print('Not connected to remote API server')
	sys.exit("No connection")
        
# Getting handles for the motors and robot
err, motorL = vrep.simxGetObjectHandle(clientID, '/PioneerP3DX/leftMotor', vrep.simx_opmode_blocking)
err, motorR = vrep.simxGetObjectHandle(clientID, '/PioneerP3DX/rightMotor', vrep.simx_opmode_blocking)
err, robot = vrep.simxGetObjectHandle(clientID, '/PioneerP3DX', vrep.simx_opmode_blocking)
# Assigning handles to the ultrasonic sensors
usensor = []
for i in range(0,16):
    err, s = vrep.simxGetObjectHandle(clientID, '/PioneerP3DX/ultrasonicSensor['+str(i)+']', vrep.simx_opmode_blocking)
    usensor.append(s)

# Sensor initialization
for i in range(16):
    err, state, point, detectedObj, detectedSurfNormVec = vrep.simxReadProximitySensor(clientID, usensor[i], vrep.simx_opmode_streaming)

ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_streaming)
ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_streaming)

#BUSCA UN MAPA EN CASO DE QUE YA SE HAYA EJECUTADO ESTE CODIGO
#SINO CREA UNO NUEVO
#if os.path.exists('map.txt'):
#    print('Mapa encontrado. cargando...')
#    occgrid = np.loadtxt('map.txt')
#    tocc = 1.0*(occgrid > 0.5)
#    occgrid[occgrid > 0.5] = 0
#else:
print('Creando nuevo mapa')
occgrid = 0.5*np.ones((100,100))
tocc = np.zeros((100,100))
t = time.time()

initt = t
niter = 0

# ARRGLO CON LOS NOMBRES DE LOS CUBOS EN LA SIMULACION
cubos=["Cuboid4","Cuboid5","Cuboid6","Cuboid7","Cuboid8","Cuboid9","Cuboid10","Cuboid11",
       "Cuboid12","Cuboid13","Cuboid14","Cuboid15","Cuboid16","Cuboid17","Cuboid18"]

for i in range(15):
    # Obtener el manejador del objeto cubo
    _, cubo_handle = vrep.simxGetObjectHandle(clientID, cubos[i], vrep.simx_opmode_blocking)

    # Generar posiciones aleatorias en X, Y, Z
    random_x = random.uniform(-4, 4)  # Rango de X: -3 a 3
    random_y = random.uniform(-4, 4)  # Rango de Y: -3 a 3
    random_z = random.uniform(0.5, 0.6)  # Rango de Z: 0.5 a 0.6 PARA QUE NO CAIGAN DEL CIELO 

    # Establecer la posición del cubo de manera aleatoria
    vrep.simxSetObjectPosition(clientID, cubo_handle, -1, [random_x, random_y, random_z], vrep.simx_opmode_oneshot)

    #time.sleep(1) #PEQUEÑA PAUSA ANTES DE EMPEZAR PARA GENERAR LOS OBJETOS ALEATORIOS

print("PONER PLAY EN EL SIMULADOR")
#TIEMPO DE LA SIMULACION O DE EXPLORACION
#APROX 300 SEG GENERA UN 40% DEL MAPA
while time.time()-t < 300:
    ret, carpos = vrep.simxGetObjectPosition(clientID, robot, -1, vrep.simx_opmode_blocking)
    
    xw = carpos[0]
    yw = carpos[1]
    xr = 50 + m.ceil(xw/0.1)
    yr = 50 - m.floor(yw/0.1)
    if xr >= 100:
        xr = 100
    if yr >= 100:
        yr = 100
    occgrid[yr-1, xr-1] = 0

    ret, carrot = vrep.simxGetObjectOrientation(clientID, robot, -1, vrep.simx_opmode_blocking)

    uread = []
    ustate = []
    upt = []
    for i in range(0,16,4):
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

           rows, cols = line(yr-1, xr-1, yo-1, xo-1)
           occgrid[rows, cols] = 0
           tocc[yo-1, xo-1] = 1

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

    ul = 1
    ur = 1
    lgains = np.linspace(0,-1,len(upt)//2)
    rgains = np.linspace(-1,0,len(upt)//2)
    for k in range(len(upt)//2):
        if ustate[k]:
            ul = ul + lgains[k]*(1.0 - uread[k])
            ur = ur + rgains[k]*(1.0 - uread[k])
    print('lvel {}   rvel {}'.format(ul, ur))

    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, ul, vrep.simx_opmode_oneshot)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, ur, vrep.simx_opmode_oneshot)

    niter = niter + 1

print(lgains)
print(rgains)
finalt = time.time()
print('Avg time per iteration ', (finalt-initt)/niter)

#TERMINANDO EL TIEMPO DE EJECUCION GUARDA EL MAPA Y LO IMPRIME
plt.imshow(tocc+occgrid)
plt.show()
np.savetxt('map.txt', tocc+occgrid)
for i in range(10):
    errf = vrep.simxSetJointTargetVelocity(clientID, motorL, 0.0, vrep.simx_opmode_oneshot)
    errf = vrep.simxSetJointTargetVelocity(clientID, motorR, 0.0, vrep.simx_opmode_oneshot)
vrep.simxStopSimulation(clientID, vrep.simx_opmode_oneshot)
