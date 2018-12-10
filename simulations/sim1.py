import sys
import time

sys.path.append('../src')
from our_rendering import *

# Execute motion 1
from motion1 import *

print path.shape
# time.sleep(5)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_yticklabels([])
ax.set_xticklabels([])
ax.set_xlabel(" ")
plt.gca().axes.get_yaxis().set_visible(False)
plt.gca().axes.get_xaxis().set_visible(False)

ax.w_zaxis.line.set_lw(0.)
ax.set_zticks([])

x = 1.
y = 1.
z = 1.
u = 1.
v = 1.
w = 1.

# quivers for vectors
q_vels = ax.quiver(x, y, z, u, v, w, length=1.0, colors='b', normalize=True, linewidth=3)
q_tor = ax.quiver(x, y, z, u, v, w, length=1.0, colors='r', normalize=True, linewidth=3)

# kuka box
verts_init = robot_points(0.0, 0.0)
kuka = Poly3DCollection(verts_init, facecolors='orange', linewidths=1, edgecolors='grey', alpha=.15)
verts4 = robot_points(0.0, 0.0)
kuka.set_verts(verts4)
ax.add_collection3d(kuka)

wheels = []
# wheel initialization
# wheels = np.empty((1, 4))
for it in xrange(4):
    verts, colors = make_wheel(0.0, 0.0, 0.0, 0.1, 0.0)
    obj = Poly3DCollection(verts, facecolor=colors)
    wheels.append(obj)
    ax.add_collection(wheels[it])

# View initialization
# ax.axis('equal')
ax.view_init(0, 30)
ax.set_xlim(0, 6)
ax.set_ylim(0, 6)
ax.set_zlim(0, 5)
ax.set_xlim(-5, 10)
ax.set_ylim(-5, 10)
ax.set_zlim(0, 5)

ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_ylabel('Y')

# Plotting path
ax.plot(path[0, :], path[1, :])

# Plotting obstacles
for it in range(obstacles.shape[1]):
    obs_verts, obst_cols = make_obstacle(obstacles[0, it], obstacles[1, it])
    obj = Poly3DCollection(obs_verts, facecolor=obst_cols)
    ax.add_collection(obj)


def animate(num, rob, whls, path):
    vel_val = 0.2 + 0.02 * num
    tor_val = 0.3 + 0.02 * num / 8.0
    # vel_val = 1.0
    wheel_vels = np.array([[vel_val], [vel_val], [vel_val], [vel_val]])
    wheel_torques = np.array([[tor_val], [tor_val], [tor_val], [tor_val]])

    robot_verts, wheels_verts = print_robot(path[0, num], path[1, num], path[1, num] * 250, rob, whls, wheel_vels,
                                            q_vels,
                                            wheel_torques, q_tor)
    # rob.set_verts(robot_verts)
    ax.view_init(30, 50 + 0.05 * num)
    ax.set_xlim(path[0, num] - 4.0, path[0, num] + 4.0)
    ax.set_ylim(path[1, num] - 4.0, path[1, num] + 4.0)
    # ax.set_zlim(0, 2)

    ax.elev = 30.
    ax.azim = -60.
    ax.dist = 5. - (0.02 * num)
    ax.dist = 5.


tic = time.time()
# wheels = np.array([wh1, wh1])
# Creating the Animation object
line_ani = animation.FuncAnimation(fig, animate, 560, fargs=(kuka, wheels, path),
                                   interval=1, blit=False)

plt.show()
toc = time.time()
print(toc - tic)
