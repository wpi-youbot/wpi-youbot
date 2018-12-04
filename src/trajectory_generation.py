import math
import numpy as np
from numpy.core.multiarray import ndarray


class Segment:
    def __init__(self, polynomial, time_range):
        self.poly = polynomial
        self.timepts = time_range


class Trajectory:
    def __init__(self):  # TODO: define constraints
        self.tr_vects = np.empty(shape=[2, 0])  # type ndarray of subsequent translation vectors
        self.wpts = np.empty(shape=[6, 0])  # waypoints
        self.times = np.empty(shape=[1, 0])  # timestampts
        self.path = np.empty(shape=[2, 0])
        self.path_lengths = np.array([0.0])

    # @classmethod
    def generatePath(self, start, target, obstacles):
        # obstacles: 2d array [2, n] of obstacle position vectors
        # start, target: initial and final robot position vectors [2, 1]

        kuka_safe_radius = 360.0
        kuka_safe_radius = 0.3600
        obstacle_safe_radius = 200.0
        obstacle_safe_radius = 0.2000
        safe_distance = kuka_safe_radius + obstacle_safe_radius

        Q = 1100000.0  # potential value:a core parameter that changes the safe zone between obstacle and the robot
        Q = 1.1  # potential value:a core parameter that changes the safe zone between obstacle and the robot
        target_force_scaling = 4.0

        step = 0.0100  # this step affects simulation position increment TODO: make [ m ]
        position = start
        self.path = np.concatenate((self.path, position), axis=1)
        num_obstacles = obstacles.shape[1]

        # as long as the distance to the target is greater than some threshold value
        while np.linalg.norm(target - position) > step * 1.25:
            current_forces = np.empty(shape=[2, 0])
            # for every column in the obstacles vector
            for obs in np.nditer(obstacles, flags=['external_loop'], order='F'):
                obstacle_distance = np.linalg.norm(obs.reshape(2, 1) - position)
                if obstacle_distance < safe_distance:
                    print "ERROR! COLLISION WITH OBSTACLE!"

                # get the unit vector of repulsing interaction force
                unit_force_vec = (position - obs.reshape(2, 1)) / obstacle_distance
                # Additional scaling factor 0.001 was introduced
                force_vec = unit_force_vec * (Q / np.power(obstacle_distance, 2))
                print unit_force_vec.shape
                force_vec = force_vec.reshape(2, 1)
                # Collecting the reaction force to all forces
                current_forces = np.concatenate((current_forces, force_vec), axis=1)

            # Calculate interaction force from the target
            target_force_vec = target_force_scaling * (target - position) / np.linalg.norm(target - position)

            # Include target force together with the rest of the forces
            current_forces = np.concatenate((current_forces, target_force_vec), axis=1)

            resulting_force = np.sum(current_forces, axis=1)
            resulting_force_unit_vec = resulting_force.reshape(2, 1) / np.linalg.norm(resulting_force)
            translation_vec = resulting_force_unit_vec * step

            position = position + translation_vec

            # update path once new position is calculated
            self.path = np.append(self.path, position)
            # increase total path length vector (used for plotting)

            self.path_lengths = np.concatenate((self.path_lengths, np.array([self.path_lengths[-1] + step])), axis=0)
            # add translation vector to the translation vectors array
            self.tr_vects = np.concatenate((self.tr_vects, translation_vec), axis=1)

    def print_dists(self):
        print self.path_lengths

    @classmethod
    def create(cls):
        return cls

    def getPolynomial(self):
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
            t_seg = np.arange(self.times[i], self.times[i + 1], self.time_inc)
            s = Segment(poly_coefficients, t_seg)
            self.seg.append(s)


if __name__ == "__main__":
    print("Hello")
    # x = np.array([[10, 0, 5], [40, 0, 5], [30, 0, 5], [90, 0, 5]])  # pos, vel, acc
    x = np.array([[10, 0, 0], [40, 0, 0], [30, 0, 0], [90, 0, 0]])  # pos, vel, acc
    t_int = np.array([0, 2, 4, 6])
    traj = Trajectory(x, t_int, 5)
    traj.getPolynomial()
    # this won't be run when imported

# print(m)


# 3-rd unused:

# m = np.array([[1.0, 0.0, 0.0, 0.0], [1, tf, tf^2, tf^3], [0, 1, 2*tf, 3*tf^2]])
# b = np.array([x[i][1], x[i][2], x[i+1][1], x[i+1][2]])
# a = m/b
