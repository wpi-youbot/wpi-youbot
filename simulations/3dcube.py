from our_rendering import *

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.set_yticklabels([])
ax.set_xticklabels([])
ax.set_xlabel(" ")
plt.gca().axes.get_yaxis().set_visible(False)
plt.gca().axes.get_xaxis().set_visible(False)

ax.w_zaxis.line.set_lw(0.)
ax.set_zticks([])

# ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
# ax.zaxis._axinfo["grid"]['color'] =  (1,1,1,0)


# plt.axis('off')

# ax.xaxis._axinfo["grid"]['color'] =  (1,1,1,0)
# ax.yaxis._axinfo["grid"]['color'] =  (1,1,1,0)# ax.w_zaxis.line.set_lw(0.)
# ax.set_zticks([])


# quivers = ax.quiver(*segs, length=0.1, colors=cols, normalize=True)

x = 1.
y = 1.
z = 1.
u = 1.
v = 1.
w = 1.

q_vels = ax.quiver(x, y, z, u, v, w, length=1.0, colors='b', normalize=True, linewidth=3)
q_tor = ax.quiver(x, y, z, u, v, w, length=1.0, colors='r', normalize=True, linewidth=3)

verts_init = robot_points(0.0, 0.0)
kuka = Poly3DCollection(verts_init, facecolors='orange', linewidths=1, edgecolors='grey', alpha=.15)

verts4 = robot_points(1.0, 1.0)
kuka.set_verts(verts4)

# verts_wheel, cols2 = make_wheel(2, 2, 2, 0.15, 0)
# wh1 = Poly3DCollection(verts_wheel, facecolor=cols2)

ax.add_collection3d(kuka)
ax.view_init(0, 30)
# ax.add_collection3d(wh1)

# ax.axis('equal')
ax.view_init(0, 30)
ax.set_xlim(0, 5)
ax.set_ylim(0, 5)
ax.set_zlim(0, 5)
ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_ylabel('Y')

wheels = []
# wheel initialization
# wheels = np.empty((1, 4))
for it in xrange(4):
    verts, colors = make_wheel(0.0, 0.0, 0.0, 0.1, 0.0)
    obj = Poly3DCollection(verts, facecolor=colors)
    wheels.append(obj)
    ax.add_collection(wheels[it])
    # ax.add_collection(obj)


# TODO: add wheel covers https://stackoverflow.com/questions/18897786/transparency-for-poly3dcollection-plot-in
#  -matplotlib

def update_lines(num, rob, whls, path):
    vel_val = 0.5 + 0.02 * num
    tor_val = 1.0 + 0.02 * num/2.0
    # vel_val = 1.0
    wheel_vels = np.array([[vel_val], [vel_val], [vel_val], [vel_val]])
    wheel_torques = np.array([[tor_val], [tor_val], [tor_val], [tor_val]])

    robot_verts, wheels_verts = print_robot(path[num], path[num], path[num] * 250, rob, whls, wheel_vels, q_vels, wheel_torques, q_tor)
    # rob.set_verts(robot_verts)
    ax.view_init(30, 50 * path[num])
    ax.set_xlim(path[num] - 4.0, path[num] + 4.0)
    ax.set_ylim(path[num] - 4.0, path[num] + 4.0)
    ax.set_zlim(0, 2)

    ax.elev = 70.
    ax.azim = -60.
    ax.dist = 5. - (0.02 * num)


path = np.arange(1.0, 5.0, 0.02)
tic = time.time()
# wheels = np.array([wh1, wh1])
# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, 100, fargs=(kuka, wheels, path),
                                   interval=10, blit=False)

plt.show()
toc = time.time()
print(toc - tic)
