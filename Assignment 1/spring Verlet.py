# uncomment the next line if running in a notebook
# %matplotlib inline
import numpy as np
import matplotlib.pyplot as plt
import time

start_time = time.time()
# mass, spring constant, initial position and velocity
m = 1
k = 1
x = 0
v = 1

# simulation time, timestep and time
t_max = 100
dt = 0.000001
t_array = np.arange(0, t_max, dt)

# initialise empty lists to record trajectories
x_list = []
v_list = []

x_list.append(x)
v_list.append(v)

# Euler integration
for t in t_array:
    a = -k * x / m
    x_list.append(x)
    v_list.append(v)
    if t == 0:
        x = x + dt * v
        v = v + dt * a
    else:
        x = 2*x - x_list[-2] + a*(dt**2)
        v = (x - x_list[-2])/(2*dt)



print("Execution Time: {}".format(time.time()-start_time))
