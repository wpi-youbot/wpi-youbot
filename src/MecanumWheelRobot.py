import numpy as np
from scipy import integrate
import matplotlib.pyplot as plt
import math

# Define KUKA YouBot properties
m0 = 20  # mass of platform (kg)
m = 20  # mass of platform (kg)
m1 = 1.40  # TODO find mass of wheels
R = 0.100/2  # radius of wheels (m) #
l = 0.150  # platform center to wheel center (m)
rho1 = 0.2355  # platform center to back axel (m)
rho2 = 0.2355  # platform center to front axel (m)

# Mass moments of inertia
J0 = (m0 / 12) * (4 * math.pow(l, 2) + math.pow(rho1 + rho2, 2))
J1 = ((m1 * math.pow(R, 2)) / 2)
J2 = ((m1 * math.pow(R, 2)) / 4)

# TODO what is m?
A = m * math.pow(R, 2) + 4 * J1
B = m1 * math.pow(R, 2) + J1
alpha = B / A

J = J0 + 4 * J2 + 2 * m1 * (math.pow(rho1, 2) + math.pow(rho2, 2) + 2 * math.pow(l, 2)) + (
            (4 * J1) / math.pow(R, 2)) * (math.pow(l + rho1, 2) + math.pow(l + rho2, 2)) - (
                (4 * B) / math.pow(R, 2)) * alpha * math.pow(rho1 - rho2, 2)


# Ode function for 4-mecanum-wheeled mobile platform
def ode_mecanum_mobile_dynamics(t, x):
    dx = np.zeros((6, 1))
    dx[0:3] = x[3:6]

    psi = x[2]
    x_c_dot = x[3]
    y_c_dot = x[4]
    psi_dot = [5]

    moment_val = 1
    M1 = moment_val
    M2 = moment_val
    M3 = moment_val
    M4 = moment_val

    # TODO need to use driving moments M(1-4) as inputs
    dx[3] = ((-4 * J1) / A) * (-1) * y_c_dot * psi_dot + (R / A) * (
                (M1 + M2 + M3 + M4) * math.cos(psi) + (M1 - M2 - M3 + M4) * math.sin(psi))
    dx[4] = ((-4 * J1) / A) * (-1) * x_c_dot * psi_dot + (R / A) * (
                (M1 + M2 + M3 + M4) * math.sin(psi) - (M1 - M2 - M3 + M4) * math.sin(psi))
    dx[5] = (1 / (J * R)) * [l * (M2 - M1 + M4 - M3) + rho2 * (M2 - M1) + rho1(M4 - M3)]

    return dx


# Main method
if __name__ == "__main__":
    t0, tf = 0, 50  # start and end time
    t = np.linspace(t0, tf, 100)  # the points of evaluation of solution

    # TODO Define start state
    # x0 = np.zeros((6, 1))
    x0 = np.zeros((6,))
    x = np.zeros((len(t), len(x0)))
    x[0, :] = x0

    # TODO Define driving moments for inputs, need to pass onto ode
    # M1, M2, M3, M4

    r = integrate.ode(ode_mecanum_mobile_dynamics).set_integrator("dopri5")  # choice of method
    r.set_initial_value(x0, t0)  # initial values
    for i in range(1, t.size):
        x[i, :] = r.integrate(t[i])  # get one more value, add it to the array
        if not r.successful():
            raise RuntimeError("Could not integrate")
    xd = x[3, :]
    plt.plot(t, xd)

    plt.show()
