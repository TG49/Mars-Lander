# uncomment the next line if running in a notebook
# %matplotlib inline
from tracemalloc import start
import numpy as np
import matplotlib.pyplot as plt
import time

start_time = time.time()
# mass, spring constant, initial position and velocity
m = 1
k = 1
x0 = 0
v0 = 1


# simulation time, timestep and time
t_max = 100
dt = 0.01
dtAnalytic = 0.001


t_array = np.arange(0, t_max, dt)
t_arrayAnalytic = np.arange(0,t_max,dtAnalytic)

#Arrays for Euler Approximation
x_numericalEuler = np.zeros(t_array.shape)
v_numericalEuler = np.zeros(t_array.shape)

#Arrays for Verlet Approximation
x_numericalVerlet = np.zeros(t_array.shape)
v_numericalVerlet = np.zeros(t_array.shape)

#Get analytical Solution
w=np.sqrt(k/m)
x_analytical = (v0/w)*np.sin(w*t_arrayAnalytic)+x0*np.cos(w*t_arrayAnalytic)
v_analytical = v0*np.cos(w*t_arrayAnalytic)-x0*w*np.sin(w*t_arrayAnalytic)


def Euler():
    #Initial Values
    x_numericalEuler[0]=x0
    v_numericalEuler[0]=v0
    # Euler integration
    for i,t in enumerate(t_array):
        #Can estimate x and v for one dt after that which is needed. This causes array to go
        #out of bounds. Ensure this does not happen
        if (i+1) < len(t_array):
            # calculate new position and velocity
            a = -k * x_numericalEuler[i] / m
            x_numericalEuler[i+1] = x_numericalEuler[i] + dt * v_numericalEuler[i]
            v_numericalEuler[i+1] = v_numericalEuler[i] + dt * a

#Using Euler to estimate the second element, as cannot run Verlet algorithm unless have two values
def Verlet():
    x_numericalVerlet[0]=x0

    for i,t in enumerate(t_array):
        if i==0:
            x_numericalVerlet[i+1]=x_numericalVerlet[0]+dt*v0
        elif (i+1)<len(t_array):
            a= -k * x_numericalVerlet[i]/m
            x_numericalVerlet[i+1]=2*x_numericalVerlet[i]-x_numericalVerlet[i-1]+a*(dt**2)
            v_numericalVerlet[i]=(1/(2*dt))*(x_numericalVerlet[i+1]-x_numericalVerlet[i-1])
        else:
            v_numericalVerlet[i] = (x_numericalVerlet[i]-x_numericalVerlet[i-1])/dt
            

#Euler()
Verlet()

print(time.time()-start_time)

fig,ax = plt.subplots(3,1)

# plot the position-time graph for Euler
plt.subplot(3,1,1)
plt.xlabel('time (s)')
plt.title("Euler")
plt.grid()
plt.plot(t_array, x_numericalEuler, label='x (m)')
plt.plot(t_array, v_numericalEuler, label='v (m/s)')
plt.legend()

plt.subplot(3,1,2)
plt.xlabel('time (s)')
plt.title("Verlet")
plt.grid()
plt.plot(t_array, x_numericalVerlet, label='x (m)')
plt.plot(t_array, v_numericalVerlet, label='v (m/s)')
plt.legend()

plt.subplot(3,1,3)
plt.xlabel('time (s)')
plt.title("Analytical")
plt.grid()
plt.plot(t_arrayAnalytic, x_analytical, label='x (m)')
plt.plot(t_arrayAnalytic, v_analytical, label='v (m/s)')
plt.legend()

plt.show()



