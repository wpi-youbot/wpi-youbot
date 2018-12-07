import numpy as np
import time
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import matplotlib


def print_robot(x, y, rot):
    # *** WHEEL ORDER AND AXIS *** TODO: update drawing with robot.py if neeeded
    #         dimension wid
    #         _________
    # dim:  1 |        | 2 # TODO: check coordinate frame with robot.py
    #    l    |   Y    |
    #    e    |   |_   |
    #    n    |      X |
    #         |        |
    #       3 __________ 4
    w = 450.0
    l = 550.0
    robot_points(x, y)
    # wheels:
    wh1_verts, _ = make_wheel(x - w / 2.0, y + l / 2, 0.1, 1)
    wh2_verts, _ = make_wheel(x + w / 2.0, y + l / 2, 0.1, 1)
    wh3_verts, _ = make_wheel(x - w / 2.0, y - l / 2, 0.1, 1)
    wh4_verts, _ = make_wheel(x + w / 2.0, y - l / 2, 0.1, 1)


def make_wheel(x, y, h, wheel_height, rot):
    nphi, nz = 23, 2
    r = 0.5  # radius of cylinder
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
