import math
import numpy as np


class Segment:
    def __init__(self, polynomial, time_range):
        self.poly = polynomial
        self.timepts = time_range


class Trajectory:
    def __init__(self, waypoints, timestampts, level):  # TODO: define constraints
        self.wpts = waypoints  # waypoints
        self.times = timestampts  # waypoints
        self.time_inc = 0.05  # [sec]
        self.level = level
        self.seg = []

    def compute_traj(self):
        # x = np.array([[10, 0], [40, 0], [30, 0], [90, 0]]) # position and velocity
        print("Range of wpts is")
        print(len(self.wpts))

        for i in range(0, len(self.wpts) - 1):
            tf = self.times[i + 1] - self.times[i]
            print("In for loop")

            m = np.array([[1.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Do not reformat this array, PyCharm keeps changing it
                          [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
                          [0.0, 0.0, 2.0, 0.0, 0.0, 0.0],  # TODO: more understaniding of 2
                          [1.0, tf, tf ^ 2, tf ^ 3, tf ^ 4, tf ^ 5],
                          [0.0, 1.0, 2.0 * tf, 3 * tf ^ 2, 4 * tf ^ 3, 5 * tf ^ 4],
                          [0.0, 0.0, 2.0, 6 * tf, 12 * tf ^ 2, 20 * tf ^ 3]])

            # b is the initial and final conditions of the segment vector
            b = np.array([self.wpts[i][0], self.wpts[i][1], self.wpts[i][2],
                          self.wpts[i + 1][0], self.wpts[i + 1][1], self.wpts[i + 1][2]])

            poly_coefficients = np.linalg.inv(m) * b.transpose()
            print(poly_coefficients)
            t_seg = np.arange(self.times[i], self.times[i+1], self.time_inc)
            s = Segment(poly_coefficients, t_seg)
            self.seg.append(s)


if __name__ == "__main__":
    print("Hello")
    # x = np.array([[10, 0, 5], [40, 0, 5], [30, 0, 5], [90, 0, 5]])  # pos, vel, acc
    x = np.array([[10, 0, 0], [40, 0, 0], [30, 0, 0], [90, 0, 0]])  # pos, vel, acc
    t_int = np.array([0, 2, 4, 6])
    traj = Trajectory(x, t_int, 5)
    traj.compute_traj()
    # this won't be run when imported

# print(m)


# 3-rd unused:

# m = np.array([[1.0, 0.0, 0.0, 0.0], [1, tf, tf^2, tf^3], [0, 1, 2*tf, 3*tf^2]])
# b = np.array([x[i][1], x[i][2], x[i+1][1], x[i+1][2]])
# a = m/b
