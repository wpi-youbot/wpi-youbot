import mpl_toolkits.mplot3d.art3d as art3d
import numpy as np
from matplotlib.patches import Circle

import time
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
# import mpl_toolkits.mplot3d *
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib


def print_robot(x, y, rot, robot, wheels, vels, q_vels, torqes, q_torques):
    """
    *** WHEEL ORDER AND AXIS *** TODO: update drawing with robot.py if neeeded

            dimension wid
             _________
     dim:  1 |        | 2 # TODO: check coordinate frame with robot.py
        l    |   Y    |
        e    |   |_   |
        n    |      X |
             |        |
           3 |________| 4
    """
    # wheel velocity vector
    print_vectors(x, y, vels, q_vels)
    print_vectors(x, y + 0.02, torqes, q_torques)

    w = 0.450
    l = 0.550

    robot_verts = robot_points(x, y)
    robot.set_verts(robot_verts)

    wheels_verts = []

    wh1_verts, _ = make_wheel(x - w / 2.0, y + l / 2, 0.1, 0.1, rot)
    wheels_verts.append(wh1_verts)
    wh2_verts, _ = make_wheel(x + w / 2.0, y + l / 2, 0.1, 0.1, rot)
    wheels_verts.append(wh2_verts)
    wh3_verts, _ = make_wheel(x - w / 2.0, y - l / 2, 0.1, 0.1, rot)
    wheels_verts.append(wh3_verts)
    wh4_verts, _ = make_wheel(x + w / 2.0, y - l / 2, 0.1, 0.1, rot)
    wheels_verts.append(wh4_verts)

    # wheels[0].set_verts(wheels_verts[0])
    for it in range(4):
        wheels[it].set_verts(wheels_verts[it])

    return robot_verts, wheels_verts


def print_vectors(x, y, vals, quivers):
    w = 0.450
    l = 0.550
    z_val = 0.1
    X = np.array([[x - w / 2.0], [x + w / 2.0], [x - w / 2.0], [x + w / 2.0]])
    Y = np.array([[y + l / 2], [y + l / 2], [y - l / 2], [y - l / 2]])
    Z = np.array([[z_val], [z_val], [z_val], [z_val]])
    v = np.array([[-1.0], [1.0], [-1.0], [1.0]])
    u = np.array([[0.0], [0.0], [0.0], [0.0]])
    w = np.array([[0.0], [0.0], [0.0], [0.0]])

    segments = quiver_data_to_segments(X, Y, Z, u, v, w, length=vals)
    quivers.set_segments(segments)


def quiver_data_to_segments(X, Y, Z, u, v, w, length=1):
    segments = (X, Y, Z, X + v * length, Y + u * length, Z + w * length)
    segments = np.array(segments).reshape(6, -1)
    return [[[x, y, z], [u, v, w]] for x, y, z, u, v, w in zip(*list(segments))]


def make_wheel(x, y, h, wheel_height, rot):
    nphi, nz = 23, 2
    r = 0.1  # radius of cylinder
    phi = np.linspace(0, 360, nphi) / 180.0 * np.pi
    z = np.linspace(- wheel_height / 2.0, wheel_height / 2.0, nz)
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

            verts_cov1 = []
            verts_cov1.append((x - wheel_height / 2.0, y, h))
            verts_cov1.append((x - wheel_height / 2.0, y, h))
            verts_cov1.append((z0 + x, sp0 + y, cp0 + h))
            verts_cov1.append((z0 + x, sp1 + y, cp1 + h))

            verts_cov2 = []
            verts_cov2.append((x + wheel_height / 2.0, y, h))
            verts_cov2.append((x + wheel_height / 2.0, y, h))
            verts_cov2.append((z1 + x, sp0 + y, cp0 + h))
            verts_cov2.append((z1 + x, sp1 + y, cp1 + h))

            verts2.append(verts)
            verts2.append(verts_cov1)
            verts2.append(verts_cov2)
            if i % 2:
                dark = 0.15
                col = (dark, dark, dark, 1)
            else:
                light = 0.99
                col = (light, light, light, 1)
            cols.append(col)
            cols.append(col)
            cols.append(col)

    return verts2, cols


def robot_points(x, y):
    r_wid = 0.35  # x axis
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


def print_obstacle(ax, radius, height, elevation=0, resolution=13, color='r', x_center=0, y_center=0):
    x = np.linspace(x_center - radius, x_center + radius, resolution)
    z = np.linspace(elevation, elevation + height, resolution)
    X, Z = np.meshgrid(x, z)

    Y = np.sqrt(radius ** 2 - (X - x_center) ** 2) + y_center  # Pythagorean theorem

    ax.plot_surface(X, Y, Z, linewidth=0, color=color)
    ax.plot_surface(X, (2 * y_center - Y), Z, linewidth=0, color=color)

    # floor = Circle((x_center, y_center), radius, color=color)
    # ax.add_patch(floor)
    # art3d.pathpatch_2d_to_3d(floor, z=elevation, zdir="z")

    # ceiling = Circle((x_center, y_center), radius, color=color)
    # ax.add_patch(ceiling)
    # art3d.pathpatch_2d_to_3d(ceiling, z=elevation + height, zdir="z")


def make_obstacle(x, y):
    nphi, nz = 43, 2
    r = 0.2  # radius of cylinder
    phi = np.linspace(0, 360, nphi) / 180.0 * np.pi
    h = 0.25
    z = np.linspace(0, h, nz)
    print z

    cols = []
    verts2 = []

    # rot = 12.0 / 180.0 * np.pi

    for i in range(len(phi) - 1):
        cp0 = r * np.cos(phi[i])
        cp1 = r * np.cos(phi[i + 1])
        sp0 = r * np.sin(phi[i])
        sp1 = r * np.sin(phi[i + 1])

        for j in range(len(z) - 1):
            z0 = z[j]
            z1 = z[j + 1]
            verts = []
            verts.append((cp0 + x, sp0 + y, z0 + h))
            verts.append((cp1 + x, sp1 + y, z0 + h))
            verts.append((cp1 + x, sp1 + y, z1 + h))
            verts.append((cp0 + x, sp0 + y, z1 + h))

            verts2.append(verts)
            if i % 2:
                dark = 0.25
                col = (dark, dark, dark, 1)
            else:
                light = 0.40
                col = (light, light, light, 1)
            cols.append(col)

    return verts2, cols
