import numpy as np
import math


class Robot:
    def __init__(self, pos):  # TODO: define constraints
        self._pos = pos  # robot position on the floor
        self._robot_r = 0.360  # safe radius [m]
        self._wheel_r = 0.145 / 2  # safe radius [m]
        self._robot_base = np.array([0.300 / 2.0, 0.471 / 2.0])  # safe radius [m]

        # it used to be this way:
        # self._pos = pos  # robot position on the floor
        # self._robot_r = robot_radius  # safe radius [m]
        # self._wheel_r = wheel_radius  # safe radius [m]
        # self._robot_base = robot_base  # safe radius [m]
        # self.base =   # robot position on the floor

        tan = math.tan(np.pi / 4)  # for the 45 deg wheel configuration
        self.J = (1.0 / self._wheel_r) * \
                 np.matrix([[1, 1 / tan, -((self._robot_base[0] * tan + self._robot_base[1]) / tan)],
                            [1, -1 / tan, ((self._robot_base[0] * tan + self._robot_base[1]) / tan)],
                            [1, -1 / tan, -((self._robot_base[0] * tan + self._robot_base[1]) / tan)],
                            [1, 1 / tan, ((self._robot_base[0] * tan + self._robot_base[1]) / tan)]])

        self.pinvJ = np.linalg.pinv(self.J)

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
        # *** WHEEL ORDER AND AXIS ***
        #         _________
        #       1 |        | 2 # TODO: check coordinate frame
        #         |   X    |
        #         |   |_   |
        #         |      Y |
        #         |        |
        #       3 __________ 4

        # Swap Vo x and y directions to match the world coordinates
        Vo = np.matrix([[vel[1]],
                        [vel[0]],
                        [vel[2]]])
        Vw = self.J * Vo
        return Vw

    def f_kins(self, wheels_vel):
        vel = self.pinvJ * wheels_vel
        # Swap Vo x and y directions to match the world coordinates
        vel = np.array([vel[1, 0],
                        vel[0, 0],
                        vel[2, 0]])
        return vel

    def f_dyn(self, wheels_torques):
        pinvJtrans = np.linalg.pinv(np.transpose(self.pinvJ))
        force = pinvJtrans * wheels_torques
        return force


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
    r1 = Robot(pos)

    cart_vel = np.array([0.3, 0.5, 0])
    print (cart_vel)
    wheels = r1.i_kins(cart_vel)
    print (wheels)

    new_wheels = r1.f_kins(np.array([[1], [0], [0], [1]]))
    lin_wheel_travel = 0.145/2
    test_lin = lin_wheel_travel * (np.sqrt(2)/2)
    print lin_wheel_travel
    print test_lin
    travel_robot = np.sqrt(np.power(new_wheels[0], 2) + np.power(new_wheels[1], 2))
    print travel_robot

    print (new_wheels)
    trq = 0.145/2
    torques = np.array([[trq], [0], [0], [trq]])
    force = r1.f_dyn(torques)
    print (force)


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
