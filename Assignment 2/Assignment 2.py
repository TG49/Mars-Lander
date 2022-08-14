import numpy as np
import matplotlib.pyplot as plt
import scipy.integrate as integrate
from scipy.spatial.transform import Rotation

###################### Initial Setup #######################
M = 6.42e23
G = 6.6743e-11
R = 3396.2e3 #Number in km

initPos = [1.5,0,0] #In terms of planetary Radius, in orthogonal directions x,y,z
initVel = [0,1,0] #ratio of orthogonal directions x,y,z
orbitalT = 50000 #Time to simulate
dt = 1 #Time step
type = "Euler" #Approximation Type to use
percentageOfCircular = 1.1 #Ratio of initial velocity to velocity required for circular orbit
scenario = 0 #Which scenario is being considered. Only need to change for scenario 1


def updateAccelerationVector(pos):
    posUnit = (1/np.linalg.norm(pos))*pos
    accel = -(G*M)/(np.linalg.norm(pos)**2) * posUnit
    return accel

#Integral to compute
def integrand(x):
    return (1/np.sqrt(2*G*M))*np.sqrt((posMagnitude*R*x)/(posMagnitude*R-x))

#With help from https://stackoverflow.com/questions/6802577/rotation-of-3d-vector
    
def transformPoints(angle,position,axis, flipAxis = 1):
    transformedPoints = np.empty_like(position)
    if np.linalg.norm(axis) != 1:
        axis = axis/np.linalg.norm(axis)
    if angle != 0:
        rotationObject = Rotation.from_rotvec(flipAxis*angle*axis)
        for i,point in enumerate(position):
            transformedPoints[i] = rotationObject.apply(point)
    else:
        transformedPoints = position
    if abs(transformedPoints[-1][2]) > 1e-3:
        print("Flip")
        transformedPoints = transformPoints(angle, position, axis, -1*flipAxis)
    
    return transformedPoints

def transformFor2D(position, normalToPlane):
    lineOfIntersect = np.cross([0,0,1],normalToPlane)
    lineOfIntersect = 1/np.linalg.norm(lineOfIntersect)*lineOfIntersect

    angle = np.arccos(np.dot([0,0,1],normalToPlane))
    if angle > np.pi/2:
        angle = np.pi-angle
    return transformPoints(angle, position, lineOfIntersect)

def PlotTrajectory2D(ax,position, normalToPlane):
    transformedPoints = transformFor2D(position, normalToPlane)
    
    xCoord = np.empty(len(transformedPoints))
    yCoord = np.empty(len(transformedPoints))
    alphas = np.empty(len(transformedPoints))
    cmap = np.empty((len(transformedPoints),3))
    
    for i,point in enumerate(transformedPoints):
        xCoord[i] = point[0]/R
        yCoord[i] = point[1]/R
        #print(1-(i/len(transformedPoints)))
        alphas[i] = 1-(i/len(transformedPoints))

        if altitude[i]<=0:
            cmap[i] = [1,1,0]
        else:
            cmap[i] = [0,0,1]
    planet = plt.Circle((0,0), radius = 1, color = "red")
    ax.add_artist(planet)
    ax.scatter(xCoord, yCoord,s=0.05,  c=cmap, alpha=alphas, label="trajectory", marker="o")
    ax.set_xlim(-5,5)
    ax.set_ylim(-5,5)
    ax.set_aspect(1)
    ax.set_xlabel("R meters")
    ax.set_ylabel("R meters")
    ax.set_title("Orbital Plane: {:.1f}x+{:.1f}y+{:.1f}z=0".format(normalToPlane[0], normalToPlane[1], normalToPlane[2]))

def PlotTrajectory3D(ax,position):
    xCoord = np.empty(len(position))
    yCoord = np.empty(len(position))
    zCoord = np.empty(len(position))
    for i,point in enumerate(position):
            xCoord[i] = point[0]/R
            yCoord[i] = point[1]/R
            zCoord[i] = point[2]/R
    ax.plot(xCoord, yCoord, zCoord, color="blue")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("3D Orbital Plot")

def plotSphere(ax):
    #Set up sphere
    phi,theta = np.mgrid[0.0:np.pi:10j, 0.0:2*np.pi:10j]
    x=np.sin(phi)*np.cos(theta)
    y=np.sin(phi)*np.sin(theta)
    z=np.cos(phi)
    ax.plot_surface(x,y,z,rstride=1,cstride=1, alpha = 0.5, color = "red")

def plotPlane(ax, normalToPlane):
    A = normalToPlane[0]
    B = normalToPlane[1]
    C = normalToPlane[2]
    D = 0

    x=np.linspace(ax.get_xlim()[0], ax.get_xlim()[1], 5)
    y=np.linspace(ax.get_ylim()[0], ax.get_ylim()[1], 5)

    X,Y = np.meshgrid(x,y)

    Z = 1/C * (D-A*X-B*Y)
    ax.plot_surface(X,Y,Z, alpha = 0.2)
    


#######################Start of program#############################################
posMagnitude = np.linalg.norm(initPos)
analyticT = integrate.quad(integrand, R, posMagnitude*R)[0]
vCircular = np.sqrt(G*M /(posMagnitude*R))
vEscape = np.sqrt(2*G*M/(posMagnitude*R))
if scenario == 1:
    velocityMagnitude = 0
else:
    velocityMagnitude=vEscape * percentageOfCircular

print(velocityMagnitude)

############ Set up Time #############
t_array = np.arange(0, orbitalT, dt)


############ Set Up iterators ############
approximateT = 0
position = np.zeros((len(t_array),3))
position[0] = np.array(initPos)*R
velocity = np.zeros((len(t_array),3))
velocity[0] = np.array(initVel)/np.linalg.norm(initVel)*velocityMagnitude
#Altitude in terms of planetary Radius
altitude = np.zeros(len(t_array))
normalToPlane = np.empty(3)

########### Perform Estimations ############
if(type == "Euler"):
    for i,t in enumerate(t_array):
        altitude[i] = np.linalg.norm(position[i])/R - 1
        if (i+1) < len(t_array): 
            accelerationVector = updateAccelerationVector(position[i])
            position[i+1] = position[i] + dt * velocity[i]
            velocity[i+1] = velocity[i] + dt * accelerationVector
        else:
            approximateT = t
            normalToPlane = np.cross(initPos,position[int(len(t_array)/2)])
            normalToPlane = 1/np.linalg.norm(normalToPlane) * normalToPlane
            break

        #Handle scenario 1 case
        if scenario == 1 and altitude[i] <= 0:
            approximateT = t
            break

elif(type == "Verlet"):
    for i,t in enumerate(t_array):
        altitude[i] = np.linalg.norm(position[i]/R) - 1
        if i==0:
            position[i+1] = position[i]+dt*velocity[i]
        elif (i+2) < len(t_array):
            accelerationVector = updateAccelerationVector(position[i])
            position[i+1] = 2*position[i]-position[i-1]+accelerationVector*(dt**2)
            velocity[i] = (1/(2*dt)) * (position[i+1]-position[i-1])
        else:
            approximateT = t
            normalToPlane = np.cross(initPos,position[int(len(t_array)/2)])
            normalToPlane = 1/np.linalg.norm(normalToPlane) * normalToPlane
            break

        #Handle scenario 1 case
        if scenario == 1 and altitude[i] <= 0:
            approximateT = t
            break
else:
    raise Exception("Please select either Euler or Verlet type")

################ Plot Graphs #######################
fig = plt.figure()

if scenario == 1:
    plt.plot(t_array, altitude, label = "Approximate Solution: t = {}".format(approximateT), color = "blue")
    plt.vlines(analyticT,ymin=0, ymax=posMagnitude-1, linestyles="dashed", label="Analytical Solution: t = {:.1f}".format(analyticT), color = "red")
    plt.grid()
    plt.legend()
    plt.title("{} Approximation of Straight Down Descent Altitude with dt = {}".format(type, dt))
else:
    ## 2D Plot ##
    ax = fig.add_subplot(1,2,1)
    PlotTrajectory2D(ax,position, normalToPlane)

    ## 3D Plot ##
    ax = fig.add_subplot(1,2,2,projection="3d")
    PlotTrajectory3D(ax,position)
    plotSphere(ax)
    plotPlane(ax, normalToPlane)

plt.show()



