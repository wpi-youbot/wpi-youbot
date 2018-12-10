import matplotlib
import matplotlib.pyplot as plt

from trajectory_generation import *
import numpy as np

mass = 20
u_friction = 0.02
# u_friction = 0.20
# u_friction = 0.10
# u_friction = 0.05
acceleration = 2.0
# acceleration = 0.25
vel_limit = 2.0
traj = Trajectory(mass, u_friction, acceleration, vel_limit)

start = np.array([[2.0],
                  [0.0]])

target = np.array([[2.0],
                   [5.0]])

obstacles = np.array([[1.7, 2.3, 3.5],
                      [1.6, 1.9, 3.5]])

traj.generate_path(start, target, obstacles)
traj.calculate_safe_vels()
path_based_vels = traj.safe_vels
# print vels
print path_based_vels.shape
# traj.print_dists()
path = traj.path
print path.shape


# print path.shape
# print vels
# print path
# time.sleep(10)

# fig, ax = plt.subplots()
# ax.plot(path[0, :], path[1, :])
# ax.set(xlabel='x', ylabel='y',
#        title='Path in the cartesian space')
# ax.grid()
# fig2, ax2 = plt.subplots()
# # ax2.plot(vels, traj.path)
# ax2.plot(vels)
# ax2.set(xlabel='Path point', ylabel='Safe velocity value',
#        title='Safe robot velocities over path waypoints')
# ax2.grid()
# plt.show()
