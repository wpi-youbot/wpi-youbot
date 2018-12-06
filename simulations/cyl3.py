from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import time

fig = plt.figure()
ax = fig.gca(projection='3d')


def make_wheel(x, y, h, rot):
    nphi, nz = 33, 2
    r = 1  # radius of cylinder
    phi = np.linspace(0, 360, nphi) / 180.0 * np.pi
    z = np.linspace(0, 1.0, nz)
    print z

    cols = []
    verts2 = []

    x = 0
    y = 0
    h = 0
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


verts_new, cols2 = make_wheel(2, 2, 2, 12)
tic = time.time()
toc = time.time()
print(toc - tic)

poly3 = Poly3DCollection(verts_new, facecolor=cols2)

poly3.set_alpha(1.0)
ax.add_collection3d(poly3)
ax.set_xlabel('X')
ax.set_xlim3d(-2, 2)
ax.set_ylabel('Y')
ax.set_ylim3d(-2, 2)
ax.set_zlabel('Z')
ax.set_zlim3d(-2, 2)
plt.show()
