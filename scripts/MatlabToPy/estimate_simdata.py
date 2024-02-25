import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import MakeRandomEz
import Sholuder_IK

# Main code
time, q, ez = MakeRandomEz.make_random_ez()

# Fix: Calculate qe using the updated q
qe_m,qe = Sholuder_IK.shoulder_ik(ez)

print("qe_m_x",qe_m[0])
print("qe_x",qe[0])
print("qe_m_y",qe_m[1])
print("qe_y",qe[1])

# Plotting
plt.close('all')

plt.figure()
plt.subplot(211)
plt.plot(time, np.rad2deg(q[0]), label='q1', linewidth=1.6)
plt.plot(time, np.rad2deg(q[1]), label='q2', linewidth=1.6)
# Fix: Use the updated qe
plt.plot(time, np.rad2deg(qe_m[0]), '--', label='qe1', linewidth=1.6)
plt.plot(time, np.rad2deg(qe_m[1]), ':', label='qe2', linewidth=1.6)
plt.legend(loc='upper right')  # Set legend to upper right
plt.ylabel('[deg]')
plt.ylim(-180, 180)
plt.title(__file__)

plt.subplot(212)
plt.plot(time, ez[0], label='ez_x', linewidth=1.6)
plt.plot(time, ez[1], label='ez_y', linewidth=1.6)
plt.plot(time, ez[2], label='ez_z', linewidth=1.6)
plt.legend(loc='upper right')  # Set legend to upper right
plt.xlabel('time [s]')

plt.show()
