import numpy as np
import matplotlib.pyplot as plt
import sys

altitude = []
desiredVelocity = []
velocity =[]

fileValues = np.loadtxt("output.txt")
for i in range(int(len(fileValues)/3)):
    altitude.append((float)(fileValues[3*i]))
    desiredVelocity.append((float)(fileValues[3*i+1]))
    velocity.append((float)(fileValues[3*i+2]))

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




