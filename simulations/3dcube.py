import numpy as np
import time
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib

# TODO: add wheel covers https://stackoverflow.com/questions/18897786/transparency-for-poly3dcollection-plot-in-matplotlib


def make_wheel(x, y, h, rot):
    nphi, nz = 13, 2
    r = 0.05  # radius of cylinder
    phi = np.linspace(0, 360, nphi) / 180.0 * np.pi
    wheel_height = 0.1
    z = np.linspace(0, wheel_height, nz)
    print z

    cols = []
    verts2 = []

    # x = 0
    # y = 0
    # h = 0
    # rot = 12.0 / 180.0 * np.pi
    rot = rot / 180.0 * np.pi

    for i in range(len(phi) - 1):
        cp0 = r * np.cos(phi[i] + rot)
        cp1 = r * np.cos(phi[i + 1] + rot)
        sp0 = r * np.sin(phi[i] + rot)
        sp1 = r * np.sin(phi[i + 1] + rot)

        for j in range(len(z) - 1):
            z0 = z[j]
            z1 = z[j + 1]
            verts = []
            # verts.append((cp0, sp0, z0))
            # verts.append((cp1, sp1, z0))
            # verts.append((cp1, sp1, z1))
            # verts.append((cp0, sp0, z1))

            verts.append((z0 + x, sp0 + y, cp0 + h))
            verts.append((z0 + x, sp1 + y, cp1 + h))
            verts.append((z1 + x, sp1 + y, cp1 + h))
            verts.append((z1 + x, sp0 + y, cp0 + h))

            verts2.append(verts)
            if i % 2:
                dark = 0.15
                col = (dark, dark, dark, 1)
            else:
                light = 0.99
                col = (light, light, light, 1)
            cols.append(col)

    return verts2, cols


def robot_points(x, y):
    r_wid = 0.45  # x axis
    r_len = 0.55  # y axis
    r_hei = 0.15  # z axis

    points = np.array([[x - r_wid / 2.0, y - r_len / 2.0, 0],
                       [x + r_wid / 2.0, y - r_len / 2.0, 0],
                       [x + r_wid / 2.0, y + r_len / 2.0, 0],
                       [x - r_wid / 2.0, y + r_len / 2.0, 0],
                       [x - r_wid / 2.0, y - r_len / 2.0, r_hei],
                       [x + r_wid / 2.0, y - r_len / 2.0, r_hei],
                       [x + r_wid / 2.0, y + r_len / 2.0, r_hei],
                       [x - r_wid / 2.0, y + r_len / 2.0, r_hei]])

    Zm = np.zeros((8, 3))

    # for i in range(8): Z[i, :] = np.dot(points[i, :], P)
    for i in range(8): Zm[i, :] = points[i, :]

    verts = [[Zm[0], Zm[1], Zm[2], Zm[3]],
             [Zm[4], Zm[5], Zm[6], Zm[7]],
             [Zm[0], Zm[1], Zm[5], Zm[4]],
             [Zm[2], Zm[3], Zm[7], Zm[6]],
             [Zm[1], Zm[2], Zm[6], Zm[5]],
             [Zm[4], Zm[7], Zm[3], Zm[0]],
             [Zm[2], Zm[3], Zm[7], Zm[6]]]

    return verts


fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# plot sides
verts_init = robot_points(0.0, 0.0)
kuka = Poly3DCollection(verts_init, facecolors='cyan', linewidths=1, edgecolors='grey', alpha=.15)

verts4 = robot_points(1.0, 1.0)
kuka.set_verts(verts4)

verts_wheel, cols2 = make_wheel(2, 2, 2, 0)
wh1 = Poly3DCollection(verts_wheel, facecolor=cols2)

ax.add_collection3d(kuka)
ax.add_collection3d(wh1)

# ax.axis('equal')
ax.set_xlim(0, 5)
ax.set_ylim(0, 5)
ax.set_zlim(0, 5)
ax.set_aspect('equal')
ax.set_xlabel('X')
ax.set_ylabel('Y')


def update_lines(num, rob, wheel, path):
    verts_kuka_new = robot_points(path[num], path[num])
    verts_wheel_new, _ = make_wheel(path[num]+0.3, path[num] + 0.2, 0.05, path[num] * 50)
    wheel.set_verts(verts_wheel_new)
    rob.set_verts(verts_kuka_new)
    # for line, data in zip(lines, dataLines):
    #     # NOTE: there is no .set_data() for 3 dim data...
    #     line.set_data(data[0:2, :num])
    #     line.set_3d_properties(data[2, :num])
    return rob, wheel


path = np.arange(1.0, 5.0, 0.02)
tic = time.time()
# Creating the Animation object
line_ani = animation.FuncAnimation(fig, update_lines, 100, fargs=(kuka, wh1, path),
                                   interval=10, blit=False)

plt.show()
toc = time.time()
print(toc - tic)
