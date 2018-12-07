from our_rendering import *

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# plot sides
verts_init = robot_points(0.0, 0.0)
kuka = Poly3DCollection(verts_init, facecolors='cyan', linewidths=1, edgecolors='grey', alpha=.15)

verts4 = robot_points(1.0, 1.0)
kuka.set_verts(verts4)

verts_wheel, cols2 = make_wheel(2, 2, 2, 0.15, 0)
wh1 = Poly3DCollection(verts_wheel, facecolor=cols2)

ax.add_collection3d(kuka)
# ax.add_collection3d(wh1)

# ax.axis('equal')
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
    verts, colors = make_wheel(0.0, 0.0, -10.0, 0.1, 0.0)
    obj = Poly3DCollection(verts, facecolor=colors)
    wheels.append(obj)
    ax.add_collection(wheels[it])


# TODO: add wheel covers https://stackoverflow.com/questions/18897786/transparency-for-poly3dcollection-plot-in
#  -matplotlib

def update_lines(num, rob, wheel, path):
    verts_kuka_new = robot_points(path[num], path[num])
    verts_wheel_new, _ = make_wheel(path[num] + 0.3, path[num] + 0.2, 0.05, 0.55, path[num] * 50)
    wheel[0].set_verts(verts_wheel_new)
    # wheel[0].
    rob.set_verts(verts_kuka_new)
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
