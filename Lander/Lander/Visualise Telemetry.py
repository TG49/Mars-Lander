import numpy as np
import matplotlib.pyplot as plt
import sys

altitude = np.loadtxt("Telemetry/altitude.txt")
#desiredVelocity = np.loadtxt("Telemetry/targetVel.txt")
#velocity = np.loadtxt("Telemetry/velocity.txt")
simulationtime = np.loadtxt("Telemetry/simtime.txt")

#fig, ax = plt.subplots()
#ax.plot(altitude, desiredVelocity, color = "red")
#ax.plot(altitude, velocity, color = "blue")

plt.figure(figsize=(20,15))
plt.plot(simulationtime, altitude, color = "red")

ax = plt.gca()
ax.set_ylabel("Altitude (m)")
ax.set_xlabel("Simulation Time (s)")
ax.set_title(' '.join(sys.argv[1:]))

plt.savefig('../../Assignment 5/Altitude vs simulation time/' + ' '.join(sys.argv[1:])+'.png')
plt.show()





