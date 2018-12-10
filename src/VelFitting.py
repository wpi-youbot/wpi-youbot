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
        self.right_nodes = []
        self.left_nodes = []
        # self.right_nodes = np.empty((1, 1), dtype=object)
        # self.left_nodes = np.empty((1, 1), dtype=object)
        self.vel_level = 0
        self.vel_path_constr = np.empty((1, 1))
        self.vel_output = np.empty((1, 1))

        # set starting points as previously activated
        act = np.zeros(path.shape[1])
        act[0] = 1  # start and end of trajectory is already acitvated
        act[-1] = 1
        self.prev_activated = act

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

        # this is the core matrix that will contain result
        self.vel_output = np.zeros(vel_path_constr.shape[0])
        self.vel_output[0] = abs_vel_start
        self.vel_output[-1] = abs_vel_end

        vel_path_constr[0] = abs_vel_start
        vel_path_constr[-1] = abs_vel_end

        self.vel_path_constr = vel_path_constr

        # Initializing the nodes
        # self.right_nodes.append(0)
        # self.left_nodes.append(path.shape[0] - 1)

        # Not using the nodes above the increment level for now
        # self.right_nodes = np.append(self.right_nodes, start)
        # self.left_nodes = np.append(self.left_nodes, end)

    def fit(self):
        # create fixed array for future iterative approach
        self.update_nodes()

    def update_nodes(self):
        self.vel_level
        # velocity increment depends on the line distance traveled (const) and acceleration ratio (const)
        # and the previous velocity (variable, known)
        # distance = V0*t + 0.5*a*t^2, where t is the time of execution of the line segment
        # thus:
        # 0 = (0.5*a)*t^2 + (V0)*t - distance
        # Computing delta -> getting time results -> selecting positive result
        distance = 0.01  # segment distance
        delta = np.power(self.vel_level,
                         2) + 4.0 * 0.5 * self.acc * distance  # adding the 4ac component since distance is substr
        t1 = (-self.vel_level - np.sqrt(delta)) / self.acc
        t2 = (-self.vel_level + np.sqrt(delta)) / self.acc
        if t1 > t2:
            t = t1
        else:
            t = t2
        if t <= 0.0:
            print "Error negative time"
            time.sleep(5)

        # print "This is time"
        # print t
        # update the reference level (compute current allowed velocity increment from the last velocity distance)

        self.vel_level = self.vel_level + self.acc * t
        # print self.vel_level

        # collect the new limit points from the path limit if found any
        # activation = np.where(self.path < self.vel_level)[0]
        # not_valid = np.isin(activation, self.prev_activated)

        new_found = (self.vel_path_constr < self.vel_level)
        # unique_now = new_found - self.prev_activated #  this doesnt work
        unique_now = np.logical_xor(new_found, self.prev_activated)

        # indices of newly activated points
        indices_now = np.where(unique_now)[0]

        # update all of the indices

        for id in indices_now:
            if self.prev_activated[id] == 0:
                self.vel_output[id] = self.vel_path_constr[id]

        # update previously activated
        self.prev_activated = np.logical_or(new_found, self.prev_activated)

        # clusters = []
        # cluster = []
        # for n in range(indices_now.shape[0]):
        #     if n == 0:
        #         cluster.append(indices_now[n])
        #     else:
        #         if indices_now[n] - indices_now[n - 1] == 1:
        #             cluster.append(indices_now[n])  # add to the current cluster
        #         else:  # close the cluster
        #             clusters.append(cluster)
        #             cluster = [indices_now[n]]  # clear working cluster
        #     if n == (indices_now.shape[0] - 1):  # when the last element
        #         clusters.append(cluster)

        # try to attach clusters to the existing nodes
        # for n in range(len(clusters)):

        # all_till_now = np.array([1, 0, 0, 0, 1, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1])
        # print all_till_now
        # print len( all_till_now )

        test = range(len(self.prev_activated) - 1)

        activated_right = np.zeros(len(self.prev_activated))
        ts = activated_right.shape
        for n in range(len(self.prev_activated) - 1):
            if n == 0:
                activated_right[n] = 1
            if self.prev_activated[n] == 1:
                activated_right[n + 1] = 1
                # calc new vel for the point found, if vel was not yet calculated
                if self.vel_output[n + 1] == 0:
                    t = time_for_segment(self.vel_output[n], self.acc)
                    plan_vel = self.vel_output[n] + self.acc * t
                    if plan_vel > self.vel_path_constr[n + 1]:  # apply constr if violation
                        self.vel_output[n + 1] = self.vel_path_constr[n + 1]
                    else:
                        self.vel_output[n + 1] = self.vel_output[n] + self.acc * t

        activated_left = np.zeros(len(self.prev_activated), dtype=bool)
        for n in reversed(range(1, len(self.prev_activated))):
            if n == len(self.prev_activated) - 1:
                activated_left[n] = 1  # fixing the right end
            if self.prev_activated[n] == 1:
                activated_left[n - 1] = 1
                if self.vel_output[n - 1] == 0:
                    t = time_for_segment(self.vel_output[n], self.acc)
                    plan_vel = self.vel_output[n] + self.acc * t
                    if plan_vel > self.vel_path_constr[n - 1]:  # apply constr if violation
                        self.vel_output[n - 1] = self.vel_path_constr[n - 1]
                    else:
                        self.vel_output[n - 1] = self.vel_output[n] + self.acc * t

        # print self.prev_activated
        after_propagation = np.logical_or(activated_right, activated_left)
        self.prev_activated = np.logical_or(after_propagation, self.prev_activated)

        # print after_propagation

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


def time_for_segment(vel_level, acc):
    # velocity increment depends on the line distance traveled (const) and acceleration ratio (const)
    # and the previous velocity (variable, known)
    # distance = V0*t + 0.5*a*t^2, where t is the time of execution of the line segment
    # thus:
    # 0 = (0.5*a)*t^2 + (V0)*t - distance
    # Computing delta -> getting time results -> selecting positive result
    distance = 0.01  # segment distance
    delta = np.power(vel_level,
                     2) + 4.0 * 0.5 * acc * distance  # adding the 4ac component since distance is substr
    t1 = (-vel_level - np.sqrt(delta)) / acc
    t2 = (-vel_level + np.sqrt(delta)) / acc
    if t1 > t2:
        t = t1
    else:
        t = t2
    if t <= 0.0:
        print "Error negative time"
        time.sleep(5)
    return t


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
    start = np.array([[0.0],  # x
                      [0.0],  # y
                      [0.0],  # rot
                      [0.0],  # xd
                      [0.0],  # yd
                      [0.0]])  # rotd

    final = np.array([[3.0],  # x
                      [0.0],  # y
                      [0.0],  # rot
                      [1.0],  # xd
                      [1.0],  # yd
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
