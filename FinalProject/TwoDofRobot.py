import numpy as np
import matplotlib.pyplot as plt
import time

# Define the lengths of the robot arm links
L1 = 1.0  # Length of the first link
L2 = 1.0  # Length of the second link

def RobotArm(dtheta1, theta1, Va1, Va2, theta2, dtheta2):
    m1 = 0.6
    m2 = 0.6
    L1 = 1.4
    L2 = 1.4
    I1 = 1
    I2 = 1
    kt1 = 0.02
    kt2 = 0.02
    kb1 = 0.02
    kb2 = 0.02
    B1 = 4
    B2 = 2
    Ra1 = 5
    Ra2 = 5
    g = 9.8

    M = np.array([
        [m1 * L1**2 + m2 * (L1**2 + 2 * L1 * L2 * np.cos(theta2) + L2**2) + I1, m2 * (L1 * L2 * np.cos(theta2) + L2**2)],
        [m2 * (L1 * L2 * np.cos(theta2) + L2**2), m2 * L2**2 + I2]
    ])

    V = np.array([
        -m2 * L1 * L2 * np.sin(theta2) * (2 * dtheta1 * dtheta2 + dtheta2**2) + (kt1 * kb1 / Ra1 + B1) * dtheta1,
        m2 * L1 * L2 * np.sin(theta2) * dtheta1**2 + (kt2 * kb2 / Ra2 + B2) * dtheta2
    ])

    G = np.array([
        (m1 + m2) * L1 * g * np.cos(theta1) + m2 * g * L2 * np.cos(theta1 + theta2),
        m2 * g * L2 * np.cos(theta1 + theta2)
    ])

    A = np.array([
        [kt1 / Ra1, 0],
        [0, kt2 / Ra2]
    ])

    Va = np.array([Va1, Va2])

    dd_theta = np.linalg.inv(M) @ (A @ Va - V - G)
    return dd_theta[0], dd_theta[1]

def plot_robot(ax, theta1, theta2):
    # Calculate the (x, y) position of the first joint
    x1 = L1 * np.cos(theta1)
    y1 = L1 * np.sin(theta1)
    
    # Calculate the (x, y) position of the end effector (second joint)
    x2 = x1 + L2 * np.cos(theta1 + theta2)
    y2 = y1 + L2 * np.sin(theta1 + theta2)
    
    # Clear the previous plot
    ax.cla()
    
    # Plot the links of the robot
    ax.plot([0, x1], [0, y1], 'ro-', lw=4)  # First link
    ax.plot([x1, x2], [y1, y2], 'bo-', lw=4)  # Second link
    
    # Set the plot limits
    ax.set_xlim(-2, 2)
    ax.set_ylim(-2, 2)
    
    # Set the aspect of the plot to be equal
    ax.set_aspect('equal')
    
    # Add grid
    ax.grid(True)

def pid_control(error, previous_error, integral, kp, ki, kd, dt):
    derivative = (error - previous_error) / dt
    integral += error * dt
    output = kp * error + ki * integral + kd * derivative
    return output, integral

# Initialize variables
theta1 = 0
dtheta1 = 0
theta2 = 0
dtheta2 = 0

# Setpoints in radians
setpoint_theta1 = np.radians(45)
setpoint_theta2 = np.radians(45)

# PID parameters for theta1
kp1 = 100
ki1 = 0.0
kd1 = 1

# PID parameters for theta2
kp2 = 100
ki2 = 0.0
kd2 = 1

# Initialize PID errors and integrals
error1 = error2 = 0
previous_error1 = previous_error2 = 0
integral1 = integral2 = 0

# Time parameters
dt = 0.01  # time step
total_time = 100  # total simulation time

# Set up the plot
plt.ion()
fig, ax = plt.subplots()

xdata, ydata1, ydata2 = [], [], []
start_time = time.time()

try:
    while time.time() - start_time < total_time:
        current_time = time.time() - start_time

        # Compute errors
        error1 = setpoint_theta1 - theta1
        error2 = setpoint_theta2 - theta2

        # PID control for Va1 and Va2
        Va1, integral1 = pid_control(error1, previous_error1, integral1, kp1, ki1, kd1, dt)
        Va2, integral2 = pid_control(error2, previous_error2, integral2, kp2, ki2, kd2, dt)

        # Update previous errors
        previous_error1 = error1
        previous_error2 = error2

        # Compute accelerations
        ddtheta1, ddtheta2 = RobotArm(dtheta1, theta1, Va1, Va2, theta2, dtheta2)

        # Update velocities and positions using Euler method
        dtheta1 += ddtheta1 * dt
        theta1 += dtheta1 * dt
        dtheta2 += ddtheta2 * dt
        theta2 += dtheta2 * dt

        # Update data for plotting
        xdata.append(current_time)
        ydata1.append(theta1)
        ydata2.append(theta2)

        # Plot the robot arm
        plot_robot(ax, theta1, theta2)

        fig.canvas.draw()
        fig.canvas.flush_events()

        time.sleep(dt)
except KeyboardInterrupt:
    pass
finally:
    plt.ioff()
    plt.show()
