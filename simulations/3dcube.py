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
    robot_verts, wheels_verts = print_robot(path[num], path[num], path[num] * 250, rob, whls)
    # rob.set_verts(robot_verts)
    ax.view_init(30, 50*path[num])
    ax.set_xlim(path[num] - 4.0, path[num] + 4.0)
    ax.set_ylim(path[num] - 4.0, path[num] + 4.0)
    ax.set_zlim(0, 2)

    ax.elev = 90.
    ax.azim = -90.
    ax.dist = 5. - (0.08 * num)
    ax.dist = 1.


    # for iter in range(4):
    #     whls[iter].set_verts(wheels_verts[iter])


    # verts_kuka_new = robot_points(path[num], path[num])
    # verts_wheel_new, _ = make_wheel(path[num] + 0.3, path[num] + 0.2, 0.05, 0.55, path[num] * 50)
    # wheels[0].set_verts(verts_wheel_new)
    # # wheel[0].
    # rob.set_verts(verts_kuka_new)
    # for line, data in zip(lines, dataLines):
    #     # NOTE: there is no .set_data() for 3 dim data...
    #     line.set_data(data[0:2, :num])
    #     line.set_3d_properties(data[2, :num])
    # return rob, wheel


path = np.arange(1.0, 5.0, 0.02)
tic = time.time()
# wheels = np.array([wh1, wh1])
# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, 100, fargs=(kuka, wheels, path),
                                   interval=10, blit=False)

plt.show()
toc = time.time()
print(toc - tic)
