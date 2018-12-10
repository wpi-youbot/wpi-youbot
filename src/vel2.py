from VelFitting import *
import sys
import time

# sys.path.append('../src')
# from our_rendering import *

# Execute motion 1
from motion1 import *

print("Testing fitting")
#
# x_path = np.linspace(0.0, 3.0, num=31)
# y_path = np.linspace(0.0, 0.0, num=31)

# path = np.vstack((x_path, y_path))
print path.shape
# time.sleep(3)
# print x_path.shape

# constr1 = np.linspace(1.6, 2.5, num=11)
# constr2 = np.linspace(1.0, 1.5, num=10)
# constr3 = np.linspace(1.5, 0.5, num=5)
# constr4 = np.linspace(0.5, 0.8, num=5)
# constraints = np.hstack((constr1, constr2, constr3, constr4))
# constraints = constraints.ravel()
# print constraints.shape
fit = VelFitting(path)
# print fit.prev_activated
# time.sleep(4)
# mass = 20  # []
# u_fr = 0.2
# acc = 2.0  # [m/s^2]
vel_lim = 2  # [m/s]

fit.set_robot_properties(mass, u_friction, acceleration)
start = np.array([[2.0],  # x
                  [0.0],  # y
                  [0.0],  # rot
                  [0.0],  # xd
                  [0.0],  # yd
                  [0.0]])  # rotd

final = np.array([[2.0],  # x
                  [5.0],  # y
                  [0.0],  # rot
                  [0.0],  # xd
                  [0.0],  # yd
                  [0.0]])  # rotd

# fit.set_vel_conditions(start, final, constraints, max_robot_vel=2.0)
fit.set_vel_conditions(start, final, path_based_vels, max_robot_vel=2.0)

limit_vels = fit.vel_path_constr
# print limit_vels
# print limit_vels[-1]

fig, ax = plt.subplots()
print path.shape
x_axis = np.linspace(0, 5.71, num=572)
ax.plot(x_axis, limit_vels)
# ax.plot(path[0, :], limit_vels)
ax.set(xlabel='path', ylabel='vel_limit',
       title='Vel limit over path')

for it in range(405):
    fit.update_nodes()
    # print fit.vel_output
    # print fit.vel_level
    # ax.axhline(y=fit.vel_level, color='r', linestyle='-')

print fit.vel_output
print fit.vel_output.shape

ax.plot(x_axis, fit.vel_output, color='red')

# plt.axhline(y=0.5, color='r', linestyle='-')

ax.grid()
plt.show()
