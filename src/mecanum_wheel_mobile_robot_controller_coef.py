import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
import math
import os
import csv
from scipy.interpolate import interp1d

# Define 0KUKA YouBot properties
m0 = 20         # mass of platform (kg)
m1 = 1.40       # find mass of wheels (kg)
m = m0 + m1     # total mass (kg)
R = 0.100/2     # radius of wheels (m) #
l = 0.150       # platform center to wheel center (m)
rho1 = 0.2355   # platform center to back axel (m)
rho2 = 0.2355   # platform center to front axel (m)


# Gain matrices for PD controller
K_P = np.asarray([[15,0,0],
                   [0,15,0],
                   [0,0,15]])

K_D = np.asarray([[0.1,0,0],
                   [0,0.1,0],
                   [0,0,0.1]])


# Initialize lists to keep track of wheel moments
M1_list = []
M2_list = []
M3_list = []
M4_list = []

t0, tf = 0.0, 20.0  # start and end time
t = np.linspace(t0, tf, 200)  # the points of evaluation of solution


# Reading in coefficients of quintic polynomials for a reference x trajectory
filename = "x_coef3.csv"
with open(filename , 'rb') as csvfile:
    reader = csv.reader(csvfile)
    a0x_list = []
    a1x_list = []
    a2x_list = []
    a3x_list = []
    a4x_list = []
    a5x_list = []
    for row in reader:
        a0,a1,a2,a3,a4,a5 = float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5])
        a0x_list.append(a0)            
        a1x_list.append(a1)
        a2x_list.append(a2)
        a3x_list.append(a3)
        a4x_list.append(a4)
        a5x_list.append(a5)


# Reading in coefficients of quintic polynomials for a reference y trajectory
filename = "y_coef3.csv"
with open(filename , 'rb') as csvfile:
    reader = csv.reader(csvfile)
    a0y_list = []
    a1y_list = []
    a2y_list = []
    a3y_list = []
    a4y_list = []
    a5y_list = []
    for row in reader:
        a0,a1,a2,a3,a4,a5 = float(row[0]),float(row[1]),float(row[2]),float(row[3]),float(row[4]),float(row[5])
        a0y_list.append(a0)            
        a1y_list.append(a1)
        a2y_list.append(a2)
        a3y_list.append(a3)
        a4y_list.append(a4)
        a5y_list.append(a5)
        


# Defining reference x,y position and velocity as functions of time        
time_interval = (tf-t0)/len(a0x_list)
def x_trajectory(time):
    # Determing which interval the time currently falls in
    interval_index = int(time / time_interval)
    if interval_index == len(a0x_list):
        interval_index -= 1
        
    # Getting the coefficients from that specific time interval
    a0 = a0x_list[interval_index]
    a1 = a1x_list[interval_index]
    a2 = a2x_list[interval_index]
    a3 = a3x_list[interval_index]
    a4 = a4x_list[interval_index]
    a5 = a5x_list[interval_index]
    
    # Mapping the time input to be within the range of 0 to time_interval
    old_start = interval_index*time_interval
    old_end = (interval_index+1)*time_interval
    new_start = 0
    new_end = time_interval
    mapper = interp1d([old_start, old_end], [new_start, new_end], fill_value="extrapolate")
    time = mapper(time)

    # Returning the value of the polynomial evaluated at that time
    return a0 + a1*time + a2*math.pow(time,2) + a3*math.pow(time,3) + a4*math.pow(time,4) + a5*math.pow(time,5)
    
def y_trajectory(time):
    # Determing which interval the time currently falls in
    interval_index = int(time / time_interval)
    if interval_index == len(a0x_list):
        interval_index -= 1
        
    # Getting the coefficients from that specific time interval
    a0 = a0y_list[interval_index]
    a1 = a1y_list[interval_index]
    a2 = a2y_list[interval_index]
    a3 = a3y_list[interval_index]
    a4 = a4y_list[interval_index]
    a5 = a5y_list[interval_index]

    # Mapping the time input to be within the range of 0 to time_interval
    mapper = interp1d([interval_index*time_interval,(interval_index+1)*time_interval], [0,time_interval], fill_value="extrapolate")
    time = mapper(time)

    # Returning the value of the polynomial evaluated at that time
    return a0 + a1*time + a2*math.pow(time,2) + a3*math.pow(time,3) + a4*math.pow(time,4) + a5*math.pow(time,5)
    
def x_velocity_trajectory(time):
    # Determing which interval the time currently falls in
    interval_index = int(time / time_interval)
    if interval_index == len(a0x_list):
        interval_index -= 1
        
    # Getting the coefficients from that specific time interval
    a0 = a0x_list[interval_index]
    a1 = a1x_list[interval_index]
    a2 = a2x_list[interval_index]
    a3 = a3x_list[interval_index]
    a4 = a4x_list[interval_index]
    a5 = a5x_list[interval_index]

    # Mapping the time input to be within the range of 0 to time_interval
    mapper = interp1d([interval_index*time_interval,(interval_index+1)*time_interval], [0,time_interval], fill_value="extrapolate")
    time = mapper(time)

    # Returning the value of the polynomial derivative evaluated at that time
    return a1 + 2*a2*time + 3*a3*math.pow(time,2) + 4*a4*math.pow(time,3) + 5*a5*math.pow(time,4)

def y_velocity_trajectory(time):
    # Determing which interval the time currently falls in
    interval_index = int(time / time_interval)
    if interval_index == len(a0x_list):
        interval_index -= 1

    # Getting the coefficients from that specific time interval
    a0 = a0y_list[interval_index]
    a1 = a1y_list[interval_index]
    a2 = a2y_list[interval_index]
    a3 = a3y_list[interval_index]
    a4 = a4y_list[interval_index]
    a5 = a5y_list[interval_index]

    # Mapping the time input to be within the range of 0 to time_interval
    mapper = interp1d([interval_index*time_interval,(interval_index+1)*time_interval], [0,time_interval], fill_value="extrapolate")
    time = mapper(time)

    # Returning the value of the polynomial derivative evaluated at that time
    return a1 + 2*a2*time + 3*a3*math.pow(time,2) + 4*a4*math.pow(time,3) + 5*a5*math.pow(time,4)
    
    
    
# Ode function for 4-mecanum-wheeled mobile platform
def ode_mecanum_mobile_dynamics(t, x):
    # Mass moments of inertia
    J0 = (m0 / 12) * (4 * math.pow(l, 2) + math.pow(rho1 + rho2, 2))
    J1 = ((m1 * math.pow(R, 2)) / 2)
    J2 = ((m1 * math.pow(R, 2)) / 4)
    
    A = m * math.pow(R, 2) + 4 * J1
    B = m1 * math.pow(R, 2) + J1
    alpha = B / A
    
    J = J0 + 4 * J2 + 2 * m1 * (math.pow(rho1, 2) + math.pow(rho2, 2) + 2 * math.pow(l, 2)) + (
                (4 * J1) / math.pow(R, 2)) * (math.pow(l + rho1, 2) + math.pow(l + rho2, 2)) - (
                    (4 * B) / math.pow(R, 2)) * alpha * math.pow(rho1 - rho2, 2)    
    
    
    # Current position and velocity of the state variable
    position = x[0:3]
    velocity = x[3:6]

    # Getting desired position from the reference trajectory
    position_desired  = np.zeros((3,))
    position_desired[0] = x_trajectory(t)
    position_desired[1] = y_trajectory(t)
    position_desired[2] = 0
    
    # Getting desired velocity from the reference trajectory
    velocity_desired  = np.zeros((3,))
    velocity_desired[0] = x_velocity_trajectory(t)
    velocity_desired[1] = y_velocity_trajectory(t)
    velocity_desired[2] = 0
    
    # Errors in current and desired position and velocity
    error_position = position - position_desired
    error_velocity = velocity - velocity_desired
    
    # Input controller is a PD controller that multiplies errors with gain matrices
    controller_input = np.dot(-1*K_P,error_position) + np.dot(-1*K_D,error_velocity)
#    print "error pos:\n", error_position
#    print "error vel:\n", error_velocity
#    print "contol:\n", controller_input
#    print "##############################################\n"
 
    # First control input involves control in the x-axis
    controller_input_x = controller_input[0]
    # Second control input involves control in the y-axis    
    controller_input_y = controller_input[1]
    
    # Using control values directly into the moments of the four wheels
    # Combination of control values gives full contribution of control in the x- and y- axes
    M1 = controller_input_x - controller_input_y
    M2 = controller_input_x + controller_input_y
    M3 = controller_input_x + controller_input_y
    M4 = controller_input_x - controller_input_y    

    # Saving all wheel moments inputs for further analysis
    M1_list.append(M1)
    M2_list.append(M2)
    M3_list.append(M3)
    M4_list.append(M4)
       
    # Retreiving components of the current state variable for further calculations
    psi = x[2]
    x_c_dot = x[3]
    y_c_dot = x[4]
    psi_dot = x[5]
    
    # Using dynamics of the mecanum wheeled mobile robot and wheel moments to calculate 
    # the linear and angular accelerations of the robot center
    x_c_double_dot = (((4 * J1) / A) * (-1) * y_c_dot * psi_dot) + (R / A) * (
                (M1 + M2 + M3 + M4) * math.cos(psi) + (M1 - M2 - M3 + M4) * math.sin(psi))
    y_c_double_dot = (((-4 * J1) / A) * (-1) * x_c_dot * psi_dot) + (R / A) * (
                (M1 + M2 + M3 + M4) * math.sin(psi) - (M1 - M2 - M3 + M4) * math.cos(psi))
    psi_double_dot = (1 / (J * R)) * (l * (M2 - M1 + M4 - M3) + (rho2*(M2 - M1)) + (rho1*(M4 - M3)))
    
    # Returning the dx variable for ODE integration
    dx = np.zeros((6,))
    dx[0:3] = x[3:6]
    dx[3] = x_c_double_dot
    dx[4] = y_c_double_dot
    dx[5] = psi_double_dot

    return dx


# Main method
if __name__ == "__main__":
    
    # Creating directory to save graphs for comparison
    current_directory = os.getcwd()
    new_directory_name = "KUKA_Trajectories_From_Coefficients"
    new_directory = os.path.join(current_directory, new_directory_name)
    if not os.path.exists(new_directory):
        os.makedirs(new_directory)
            
    # Initializing intial state of the system. Can be changed.
    x0 = np.zeros((6,))
    x0[0] = 2
    x0[1] = 0

    # Structure to hold all states of the system over time
    x = np.zeros((len(t), len(x0)))
    x[0, :] = x0

    # List of payloads (kg) to add to the dyanmics of the system
    payloads = [0, 20, 40, 60, 80, 100]
    
    # Loop over the number of payloads and plot the trajectories of the controller against
    # the true referenced trajectories
    for i in range (0,len(payloads)):
        
        # Adding payload to system dynamics
        global m
        m = m + payloads[i]
        #print "Current total mass:", m
        
        # ODE function that takes the first order state space equation of the dynamic model of the
        # mecanum wheeled mobile robot and computes the next state x by integrating the previous dx
        r = integrate.ode(ode_mecanum_mobile_dynamics).set_integrator("dopri5")  # choice of method
        r.set_initial_value(x0, t0)  # initial values
        for n in range(1, t.size):
            x[n, :] = r.integrate(t[n])  # get one more value, add it to the array
            if not r.successful():
                raise RuntimeError("Could not integrate")
                
    
        # Retreiving all linear and angular positions and velocities from the computed states over time
        x_pos = x[:,0]
        y_pos = x[:,1]
        psi_pos = x[:,2]
        x_vel = x[:,3]
        y_vel = x[:,4]
        psi_vel = x[:,5]
        
        # The true reference trajectory over time
        x_reference = [x_trajectory(ti) for ti in t]
        y_reference = [y_trajectory(ti) for ti in t]
        x_velocity_reference = [x_velocity_trajectory(ti) for ti in t]
        y_velocity_reference = [y_velocity_trajectory(ti) for ti in t]
        
        
        # Saving experiments for payload = 0 and payload = 100 for animation
        if i == 0:
            np.save('../simulations/x_pos.npy', x_pos)
            np.save('../simulations/y_pos.npy', y_pos)
            np.save('../simulations/x_ref.npy', x_reference)
            np.save('../simulations/y_ref.npy', y_reference)
            print "Saved np arrays for controller with payload 0"
            
        if i == len(payloads)-1:
            np.save('../simulations/x_pos_100.npy', x_pos)
            np.save('../simulations/y_pos_100.npy', y_pos)
            print "Saved np arrays for controller with payload 100"
    
        # Plotting and saving controller vs reference trajectory in new directory
        fig1 = plt.figure(1)
        plt.plot(x_reference, y_reference, 'b', label='reference traj')
        plt.plot(x_pos, y_pos, '--r', label='controller traj', linewidth=3.0)
        title = "KUKA YouBot Controller vs. Reference at m = %05.1f kg"%m 
        plt.title(title)
        plt.xlabel("x (meters)")
        plt.ylabel("y (meters)")
        plt.legend(loc='upper right')    
        plt.show()
        title = title + ".jpg"
        fig_path = os.path.join(new_directory,title)
        fig1.savefig(fig_path)
        
        # Plotting and saving x position trajectory over time in new directory
        fig2 = plt.figure(2)
        plt.plot(t, x_reference, 'b', label = 'x reference')
        plt.plot(t, x_pos, '--r', label='x controller', linewidth=3.0)
        title = "KUKA YouBot X Position over time at m = %05.1f kg"%m 
        plt.title(title)
        plt.xlabel("time")
        plt.ylabel("x (meters)")
        plt.legend(loc='upper right')    
        plt.show()        
        title = title + ".jpg"
        fig_path = os.path.join(new_directory,title)
        fig2.savefig(fig_path)
        
        # Plotting and saving y position trajectory over time in new directory
        fig2 = plt.figure(3)
        plt.plot(t, y_reference, 'b', label = 'x reference')
        plt.plot(t, y_pos, '--r', label='x controller', linewidth=3.0)
        title = "KUKA YouBot Y Position over time at m = %05.1f kg"%m 
        plt.title(title)
        plt.xlabel("time")
        plt.ylabel("y (meters)")
        plt.legend(loc='upper right')    
        plt.show()        
        title = title + ".jpg"
        fig_path = os.path.join(new_directory,title)
        fig2.savefig(fig_path)
        
        # Plotting and saving x velocity trajectory over time in new directory
        fig2 = plt.figure(4)
        plt.plot(t, x_velocity_reference, 'b', label = 'x reference')
        plt.plot(t, x_vel, '--r', label='x controller', linewidth=3.0)
        title = "KUKA YouBot X Velocity over time at m = %05.1f kg"%m 
        plt.title(title)
        plt.xlabel("time")
        plt.ylabel("x_vel (meters/time)")
        plt.legend(loc='upper right')    
        plt.show()        
        title = title + ".jpg"
        fig_path = os.path.join(new_directory,title)
        fig2.savefig(fig_path)
        
        # Plotting and saving y velocity trajectory over time in new directory
        fig2 = plt.figure(5)
        plt.plot(t, y_velocity_reference, 'b', label = 'x reference')
        plt.plot(t, y_vel, '--r', label='x controller', linewidth=3.0)
        title = "KUKA YouBot Y Velocity over time at m = %05.1f kg"%m 
        plt.title(title)
        plt.xlabel("time")
        plt.ylabel("y_vel (meters/time)")
        plt.legend(loc='upper right')    
        plt.show()        
        title = title + ".jpg"
        fig_path = os.path.join(new_directory,title)
        fig2.savefig(fig_path)
        
        
        # Resetting the current payload of the system to its original base weight
        global m, m0, m1
        m = m0 + m1
        