import numpy as np
import matplotlib.pyplot as plt
import sys

altitude = np.loadtxt("Telemetry/altitude.txt")
desiredVelocity = np.loadtxt("Telemetry/targetVel.txt")
velocity = np.loadtxt("Telemetry/velocity.txt")
#simulationtime = np.loadtxt("Telemetry/simtime.txt")

#fig, ax = plt.subplots()
#ax.plot(altitude, desiredVelocity, color = "red")
#ax.plot(altitude, velocity, color = "blue")

plt.figure(figsize=(20,15))
plt.plot(altitude, desiredVelocity, color = "blue", label = "Target Velocity")
plt.plot(altitude, velocity, color = "red", label = "Velocity")

plt.legend()
ax = plt.gca()
ax.set_ylabel("Velocity (m/s)")
ax.set_xlabel("Simulation Time (s)")
ax.set_xlim(ax.get_xlim()[::-1])
ax.set_title(' '.join(sys.argv[1:]))

plt.savefig('../../Assignment 5/Velocity vs Altitude/' + ' '.join(sys.argv[1:])+'.png')
plt.show()





