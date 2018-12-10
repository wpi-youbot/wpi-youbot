from VelFitting import *

print("Testing fitting")

x_path = np.linspace(0.0, 3.0, num=301)
y_path = np.linspace(0.0, 0.0, num=301)

path = np.vstack((x_path, y_path))
print path.shape
# time.sleep(3)
# print x_path.shape

constr1 = np.linspace(1.6, 2.5, num=101)
constr2 = np.linspace(1.0, 1.5, num=100)
constr3 = np.linspace(1.5, 0.5, num=50)
constr4 = np.linspace(0.5, 0.8, num=50)
constraints = np.hstack((constr1, constr2, constr3, constr4))
print constraints.shape

fit = VelFitting(path)
print fit.prev_activated
# time.sleep(4)
mass = 20  # []
u_fr = 0.2
acc = 1  # [m/s^2]
vel_lim = 2  # [m/s]

fit.set_robot_properties(mass, u_fr, acc)
start = np.array([[0.0],  # x
                  [0.0],  # y
                  [0.0],  # rot
                  [0.0],  # xd
                  [0.0],  # yd
                  [0.0]])  # rotd

final = np.array([[3.0],  # x
                  [0.0],  # y
                  [0.0],  # rot
                  [0.0],  # xd
                  [0.0],  # yd
                  [0.0]])  # rotd

fit.set_vel_conditions(start, final, constraints, max_robot_vel=2.0)

limit_vels = fit.vel_path_constr
print limit_vels
print limit_vels[-1]

fig, ax = plt.subplots()
print path.shape
ax.plot(path[0, :], limit_vels)

for it in range(125):
    fit.update_nodes()
    # print fit.vel_level
    # ax.axhline(y=fit.vel_level, color='r', linestyle='-')

# print fit.vel_output
ax.plot(path[0, :], fit.vel_output, color='red')

# plt.axhline(y=0.5, color='r', linestyle='-')

ax.set(xlabel='path', ylabel='vel_limit',
       title='Vel limit over path')
ax.grid()
plt.show()
