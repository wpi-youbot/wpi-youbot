import time

from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import axes3d
from matplotlib.animation import FuncAnimation
import numpy as np

fig = plt.figure()
ax = fig.gca(projection='3d')

num_frames = 50
theta = np.linspace(0, 2 * np.pi, 10, endpoint=False)
r = np.arange(1, 2.1)
z = np.arange(-2, 2.1, 1)


def compute_segs(i):
    offset = 2 * i * np.pi / num_frames
    theta2, r2, z2 = np.meshgrid(theta + offset, r, z)

    x = r2 * np.cos(theta2)
    y = r2 * np.sin(theta2)

    u = x + 0.2 * np.cos(4 * theta2)
    v = y
    w = z2 + 0.2 * np.sign(z2) * np.sin(4 * theta2)

    x = np.array([1, 1])
    y = np.array([1, 1])
    z2 = np.array([1, 1])
    u = np.array([1, 1])
    v = np.array([1, 1])
    w = np.array([1, 1])

    return x, y, z2, u, v, w


segs = compute_segs(0)
cols = ['b' for x in segs[0].ravel()]
cols[0] = 'r'
# quivers = ax.quiver(*segs, length=0.1, colors=cols, normalize=True)

x = 1.
y = 1.
z = 1.
u = 1.
v = 1.
w = 1.

quivers = ax.quiver(x, y, z, u, v, w, length=1.0, colors=cols, normalize=True, linewidth=7)
quivers2 = ax.quiver(x, y, z, u, v, w, length=1.0, colors=cols, normalize=True)

ax.set_xlim([-3, 3])
ax.set_ylim([-3, 3])
ax.set_zlim([-3, 3])


def plot_vectors(x,y, vals)
    w = 0.450
    l = 0.550
    z_val = 0.1
    X = np.array([[x - w / 2.0], [x + w / 2.0], [x - w / 2.0], [x + w / 2.0]])
    Y = np.array([[y + l / 2], [y + l / 2], [y - l / 2], [y - l / 2]])
    Z = np.array([[z_val], [z_val], [z_val], [z_val], [z_val], [z_val]])





    ,


      0.1,
      0.1,
      0.1
      0.1


def animate(i):
    segs = np.array(compute_segs(i)).reshape(6, -1)

    X = np.array([[1.], [1.5]])
    Y = np.array([[1.5], [1.5]])
    Z = np.array([[1.5], [1.5]])
    u = np.array([[1.5], [1.5]])
    v = np.array([[4.], [1.5]])
    w = np.array([[4.], [1.5]])
    len = np.array([[0.3], [0.5]])

    X2 = np.array([[1.], [1.5]])
    Y2 = np.array([[1.5], [1.5]])
    Z2 = np.array([[-1.5], [-1.5]])
    u2 = np.array([[1.5], [-1.5]])
    v2 = np.array([[4.], [1.5]])
    w2 = np.array([[4.], [1.5]])
    len2 = np.array([[0.3], [0.5]])

    segments = quiver_data_to_segments(X, Y, Z, u, v, w, length=len)
    segments2 = quiver_data_to_segments(X2, Y2, Z2, u2, v2, w2, length=len2)
    # quiver_plot.set_segments(segments)
    # data = np.array([[x, y, z], [u, v, w])

    quivers.set_segments(segments)
    quivers2.set_segments(segments2)
    return quivers


def quiver_data_to_segments(X, Y, Z, u, v, w, length=1):
    segments = (X, Y, Z, X + v * length, Y + u * length, Z + w * length)
    segments = np.array(segments).reshape(6, -1)
    return [[[x, y, z], [u, v, w]] for x, y, z, u, v, w in zip(*list(segments))]


# i = 5
# offset = 2 * i * np.pi / num_frames
#
# theta2, r2, z2 = np.meshgrid(theta + offset, r, z)
#
# x = r2 * np.cos(theta2)
# y = r2 * np.sin(theta2)
#
#
# u = x + 0.2 * np.cos(4 * theta2)
# v = y
# w = z2 + 0.2 * np.sign(z2) * np.sin(4 * theta2)

# print  '****** this type checking*******'
# print 'x type'
# print type(x)
# print x.shape
# print 'y type'
# print type(y)
# time.sleep(3)
#
# segs = np.array(compute_segs(2)).reshape(6, -1)
# print  'theese are old segs'
# print segs[0].shape
# print type(segs[0])
# time.sleep(3)

# new_segs = [[[x, y, z], [u, v, w]] for x, y, z, u, v, w in zip(*segs.tolist())]
# print  'theese are new segs'
# print new_segs
# print type(new_segs)
# time.sleep(3)

ani = FuncAnimation(fig, animate, frames=num_frames, interval=30, blit=False)
# ani.save('update_3d_quiver.gif', writer='imagemagick')

plt.show()
