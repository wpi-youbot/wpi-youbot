import matplotlib
import matplotlib.pyplot as plt
# Compute constraints and fit velocities
import numpy as np
import time


class VelFitting:
    def __init__(self, path):
        self.path = path
        self.mass = None
        self.u_friction = None
        self.acc = None
        self.max_robot_vel = None
        self.vel_limit = None
        self.right_nodes = np.empty((1, 1), dtype=object)
        self.left_nodes = np.empty((1, 1), dtype=object)
        self.vel_level = 0
        self.vel_path_constr = np.empty((1, 1))

    def set_robot_properties(self, mass, u_friction, acceleration):
        self.mass = mass
        self.u_friction = u_friction
        self.acc = acceleration

    def set_vel_conditions(self, start, end, vel_path_constr, max_robot_vel):
        """

        :type vel_path_constr: 1 dim numpy array of abs velocities
        """
        # include the boundary conditions to the path-related velocity limits
        self.max_robot_vel = max_robot_vel

        # Limit the speed allowed to max robot speed
        vel_path_constr[vel_path_constr > max_robot_vel] = max_robot_vel

        # get the absolute robot velocities for boundary conditions
        abs_vel_start = np.linalg.norm(start[3:6])
        abs_vel_end = np.linalg.norm(end[3:6])

        vel_path_constr[0] = abs_vel_start
        vel_path_constr[-1] = abs_vel_end

        self.vel_path_constr = vel_path_constr
        # Not using the nodes above the increment level for now
        # self.right_nodes = np.append(self.right_nodes, start)
        # self.left_nodes = np.append(self.left_nodes, end)

    def fit(self):
        # create fixed array for future iterative approach
        self.update_nodes()

    def update_nodes(self):
        # update the reference level (compute current allowed velocity increment from the last velocity distance)
        # collect the new limit points from the path limit if found any
        # init nodes at the boundaries of the new groups
        # exclude new points found from future search: add them to fixed points

        # start with right nodes (progressing to the right)
        # for each right node from the group
        # check the distance to the closest left node (which has different index - to avoid closing newly initialized)

        # if distance to the nearest is > 1:
            # Progress the node:
                # Compute new velocity;
                    # Check if the max new velocity is in range of the path velocity limit
                        # When in range: update the absolute velocity of the new node point
                        # Not in range: compute the allowed acceleration that will fit the limit imposed
                            # If acceleration computed is out of scope, throw an error and interrupt

                # Update node index
                # Add the node point index to fixed

        # if distance is 1:
            # check if connection between the nodes is dynamically feasible
                # if feasible, deactivate both nodes
                # if not feasible, select the node with lower velocity value
                    # see the range of indices of the node considered previously
                    # while newly planned waypoint in range of the other node:
                        # progress this node using max acceleration possible
                        # update the state of waypoint that is attached
                        # check if the new node endpoint matches with next fixed point
                        # if matches, deactivate the node and update the index of the other node
        pass


class Node:
    def __init__(self, index, velocity, direction, max_acc):
        self.id = index
        self.dir = direction
        self.velocity = 0
        self.velocity = max_acc

    def update(self):
        self.id = self.id + self.dir
        self.velocity = self.velocity  # TODO: + acc to vel incr formula

    @property
    def vel_path_constr(self):  # getter
        return self.vel_path_constr


if __name__ == "__main__":
    print("Testing fitting")

    x_path = np.linspace(0.0, 3.0, num=301)
    y_path = np.linspace(0.0, 0.0, num=301)

    path = np.vstack((x_path, y_path))
    print path.shape
    # print x_path.shape

    constr1 = np.linspace(2.0, 2.0, num=101)
    constr2 = np.linspace(1.0, 1.0, num=100)
    constr3 = np.linspace(1.5, 1.5, num=50)
    constr4 = np.linspace(0.5, 0.5, num=50)
    constraints = np.hstack((constr1, constr2, constr3, constr4))
    print constraints.shape

    fit = VelFitting(path)
    mass = 20  # []
    u_fr = 0.2
    acc = 1  # [m/s^2]
    vel_lim = 2  # [m/s]

    fit.set_robot_properties(mass, u_fr, acc)
    start = np.array([[0.0],   # x
                      [0.0],   # y
                      [0.0],   # rot
                      [0.0],   # xd
                      [0.0],   # yd
                      [0.0]])  # rotd

    final = np.array([[3.0],   # x
                      [0.0],   # y
                      [0.0],   # rot
                      [1.0],   # xd
                      [1.0],   # yd
                      [0.0]])  # rotd

    fit.set_vel_conditions(start, final, constraints, max_robot_vel=2.0)

    limit_vels = fit.vel_path_constr
    print limit_vels
    print limit_vels[-1]

    fig, ax = plt.subplots()
    print path.shape
    ax.plot(path[0, :], limit_vels)

    ax.set(xlabel='path', ylabel='vel_limit',
           title='Vel limit over path')
    ax.grid()
    plt.show()

