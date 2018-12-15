from trajectory_generation import *
import numpy as np

# Robot states
start_state = np.array([[2.0],  # x
                        [0.0],  # y
                        [0.0],  # rot
                        [0.0],  # xd
                        [0.0],  # yd
                        [0.0]])  # rotd

final_state = np.array([[2.0],  # x
                        [5.0],  # y
                        [0.0],  # rot
                        [0.0],  # xd
                        [0.0],  # yd
                        [0.0]])  # rotd

# Setting world constraints
start = np.array([[2.0],
                  [0.0]])

target = np.array([[2.0],
                   [5.0]])

obstacles = np.array([[1.7, 2.3, 3.5],
                      [1.6, 1.9, 3.5]])

# setting robot properties
u_friction = 0.3
vel_limit = 2.0

# ***** CONFIGURATION 1 *****
mass = 20.0
acceleration = 2.0

traj = Trajectory(mass, u_friction, acceleration, vel_limit)
traj.generate_path(start, target, obstacles)
traj.calculate_safe_vels()
path_based_vels = traj.safe_vels
path = traj.path

fit = VelFitting(path)
fit.set_robot_properties(mass, u_friction, acceleration)

fit.set_vel_conditions(start_state, final_state, path_based_vels, max_robot_vel=2.0)
limit_vels = fit.vel_path_constr

fig, ax = plt.subplots()
print path.shape
x_axis = np.linspace(0, 5.71, num=572)
ax.plot(x_axis, limit_vels, color='black')
ax.set(xlabel='path [m]', ylabel='velocity [m/s]',
       title='Velocity over path')

for it in range(405):
    fit.update_nodes()

print fit.vel_output
print fit.vel_output.shape
ax.plot(x_axis, fit.vel_output, color='red')

# ***** CONFIGURATION 2 *****
mass = 40.0
acceleration = 1.0

traj = Trajectory(mass, u_friction, acceleration, vel_limit)
traj.generate_path(start, target, obstacles)
traj.calculate_safe_vels()
path_based_vels = traj.safe_vels
path = traj.path

fit2 = VelFitting(path)
fit2.set_robot_properties(mass, u_friction, acceleration)

fit2.set_vel_conditions(start_state, final_state, path_based_vels, max_robot_vel=2.0)
limit_vels = fit2.vel_path_constr

# fig, ax = plt.subplots()
print path.shape
x_axis = np.linspace(0, 5.71, num=572)
# ax.plot(x_axis, limit_vels)
# ax.set(xlabel='path', ylabel='vel_limit',
#        title='Velocity limit over path')

for it in range(405):
    fit2.update_nodes()

print fit2.vel_output
print fit2.vel_output.shape
ax.plot(x_axis, fit2.vel_output, color='blue')
ax.grid()

# ***** CONFIGURATION 3 *****
mass = 80.0
acceleration = 0.50

traj = Trajectory(mass, u_friction, acceleration, vel_limit)
traj.generate_path(start, target, obstacles)
traj.calculate_safe_vels()
path_based_vels = traj.safe_vels
path = traj.path

fit3 = VelFitting(path)
fit3.set_robot_properties(mass, u_friction, acceleration)

fit3.set_vel_conditions(start_state, final_state, path_based_vels, max_robot_vel=2.0)
limit_vels = fit3.vel_path_constr

# fig, ax = plt.subplots()
print path.shape
x_axis = np.linspace(0, 5.71, num=572)
# ax.plot(x_axis, limit_vels)
# ax.set(xlabel='path', ylabel='vel_limit',
#        title='Velocity limit over path')

for it in range(405):
    fit3.update_nodes()

print fit3.vel_output
print fit3.vel_output.shape
ax.plot(x_axis, fit3.vel_output, color='orange')
ax.grid()

# ***** CONFIGURATION 4 *****
mass = 160.0
acceleration = 0.25

traj = Trajectory(mass, u_friction, acceleration, vel_limit)
traj.generate_path(start, target, obstacles)
traj.calculate_safe_vels()
path_based_vels = traj.safe_vels
path = traj.path

fit4 = VelFitting(path)
fit4.set_robot_properties(mass, u_friction, acceleration)

fit4.set_vel_conditions(start_state, final_state, path_based_vels, max_robot_vel=2.0)
limit_vels = fit4.vel_path_constr

# fig, ax = plt.subplots()
print path.shape
x_axis = np.linspace(0, 5.71, num=572)
# ax.plot(x_axis, limit_vels)
# ax.set(xlabel='path', ylabel='vel_limit',
#        title='Velocity limit over path')

for it in range(405):
    fit4.update_nodes()

print fit4.vel_output
print fit4.vel_output.shape
ax.plot(x_axis, fit4.vel_output, color='green')
ax.grid()

# ***** CONFIGURATION 5 *****
mass = 20.0
acceleration = 0.025

traj = Trajectory(mass, u_friction, acceleration, vel_limit)
traj.generate_path(start, target, obstacles)
traj.calculate_safe_vels()
path_based_vels = traj.safe_vels
path = traj.path

fit5 = VelFitting(path)
fit5.set_robot_properties(mass, u_friction, acceleration)

fit5.set_vel_conditions(start_state, final_state, path_based_vels, max_robot_vel=2.0)
limit_vels = fit5.vel_path_constr

# fig, ax = plt.subplots()
print path.shape
x_axis = np.linspace(0, 5.71, num=572)
# ax.plot(x_axis, limit_vels)
# ax.set(xlabel='path', ylabel='vel_limit',
#        title='Velocity limit over path')

for it in range(405):
    fit5.update_nodes()

print fit5.vel_output
print fit5.vel_output.shape
ax.plot(x_axis, fit5.vel_output, color='gray')
ax.grid()

plt.show()