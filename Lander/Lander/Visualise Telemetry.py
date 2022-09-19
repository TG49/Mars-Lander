import numpy as np
import matplotlib.pyplot as plt
import sys

altitude = np.loadtxt("Telemetry/altitude.txt")
desiredVelocity = np.loadtxt("Telemetry/targetVel.txt")
velocity = np.loadtxt("Telemetry/velocity.txt")
simulationtime = np.loadtxt("Telemetry/simtime.txt")

#fig, ax = plt.subplots()
#ax.plot(altitude, desiredVelocity, color = "red")
#ax.plot(altitude, velocity, color = "blue")

plt.plot(altitude, desiredVelocity, color = "red", label = "Target Velocity")
plt.plot(altitude, velocity, color = "blue", label = "Velocity")
plt.legend()

ax = plt.gca()
ax.set_xlim(ax.get_xlim()[::-1])
ax.set_ylabel("Velocity (m/s)")
ax.set_xlabel("Altitude (m)")
ax.set_title(' '.join(sys.argv[1:]))

plt.show()




