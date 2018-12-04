# Compute constraints and fit velocities
class VelFitting:
    def __init__(self, path, time_range):
        self.path = path
        self.mass = None
        self.u_friction = None
        self.acc = None
        self.vel_limit = None

    def set_robot_properties(self, mass, u_friction, acceleration, vel_limit):
        self.mass = mass
        self.u_friction = u_friction
        self.acc = acceleration
        self.vel_limit = vel_limit


class Node:
    def __init__(self, index, velocity, direction, max_acc):
        self.id = index
        self.dir = direction
        self.velocity = 0
        self.velocity = max_acc

    def update(self):
        self.id = self.id + self.dir
        self.velocity = self.velocity  # TODO: + acc to vel incr formula

