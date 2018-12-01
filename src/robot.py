import numpy as np
import math


class Robot:
    def __init__(self, pos, robot_radius, wheel_radius, robot_base):  # TODO: define constraints
        self._pos = pos  # robot position on the floor
        self._robot_r = robot_radius  # safe radius [m]
        self._wheel_r = wheel_radius  # safe radius [m]
        self._robot_base = robot_base  # safe radius [m]
        # self.base =   # robot position on the floor

        tan = math.tan(np.pi / 4)  # for the 45 deg wheel configuration
        self.J = (1.0 / self._wheel_r) * \
                 np.matrix([[1, 1 / tan, -((self._robot_base[0] * tan + self._robot_base[1]) / tan)],
                            [1, -1 / tan, ((self._robot_base[0] * tan + self._robot_base[1]) / tan)],
                            [1, -1 / tan, -((self._robot_base[0] * tan + self._robot_base[1]) / tan)],
                            [1, 1 / tan, ((self._robot_base[0] * tan + self._robot_base[1]) / tan)]])

    @property
    def pos(self):  # getter
        return self._pos

    @pos.setter
    def pos(self, value):
        self._pos = value

    @pos.deleter
    def pos(self):
        print("deleter of x called")
        del self._x

    def compute_traj(self):
        print("Hello")

    def i_kins(self, vel):
        # Swap Vo x and y directions to match the world coordinates
        Vo = np.matrix([[vel[1]],
                        [vel[0]],
                        [vel[2]]])

        print (self.J)
        Vw = self.J * Vo
        print (Vw)
        return Vw
        # return Vw


class C(object):
    def __init__(self):
        self._x = None

    @property
    def x(self):
        """I'm the 'x' property."""
        print("getter of x called")
        return self._x

    @x.setter
    def x(self, value):
        print("setter of x called")
        self._x = value

    @x.deleter
    def x(self):
        print("deleter of x called")
        del self._x


if __name__ == "__main__":
    print("This is main")

    pos = np.matrix([10, 20])
    wheel_r = 0.145 / 2
    robot_r = 0.360
    robot_b = np.array([0.300 / 2.0, 0.471 / 2.0])
    r1 = Robot(pos, robot_r, wheel_r, robot_b)
    new = np.matrix([[37],
                     [27]])
    # r1.pos = new
    # out = r1.pos
    # # print(out)
    # hel = np.array([4, 5])
    # res = new*hel
    # print(hel[0])
    # print(res)
    # print(res[1, 1])
    cart_vel = np.array([0.3, 0.5, 0])
    r1.i_kins(cart_vel)

    # # x = np.array([[10, 0, 5], [40, 0, 5], [30, 0, 5], [90, 0, 5]])  # pos, vel, acc
    # x = np.array([[10, 0, 0], [40, 0, 0], [30, 0, 0], [90, 0, 0]])  # pos, vel, acc
    # t_int = np.array([0, 2, 4, 6])
    # p = np.mat([10, 20])
    # r1 = Robot(p)
    # r1.
    # traj.compute_traj()
    # this won't be run when imported

# print(m)


# 3-rd unused:

# m = np.array([[1.0, 0.0, 0.0, 0.0], [1, tf, tf^2, tf^3], [0, 1, 2*tf, 3*tf^2]])
# b = np.array([x[i][1], x[i][2], x[i+1][1], x[i+1][2]])
# a = m/b
