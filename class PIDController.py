# import numpy as np

# class PIDController:
#     def __init__(self, Kp, Ki, Kd):
#         self.Kp = Kp
#         self.Ki = Ki
#         self.Kd = Kd
#         self.integral_error = 0.0
#         self.previous_error = 0.0

#     def update(self, error, dt):
#         # Proportional term
#         p_term = self.Kp * error

#         # Integral term
#         self.integral_error += error * dt
#         i_term = self.Ki * self.integral_error

#         # Derivative term
#         derivative_error = (error - self.previous_error) / dt
#         d_term = self.Kd * derivative_error

#         self.previous_error = error

#         return p_term + i_term + d_term
    
# def quadcopter_dynamics(t, x, T1, T2, T3, T4, m, I, L, k_d):
#     # 1. Unpack all 12 states from x
#     #    pos = x[0:3], angles = x[3:6], vel = x[6:9], rates = x[9:12]

#     # 2. Calculate total thrust T and torques [tau_phi, tau_theta, tau_psi]
#     #    from T1, T2, T3, T4 using the *forward* mixing matrix.
#     T = T1 + T2 + T3 + T4
#     tau_phi = L * (T1 - T3)
#     # ... etc.

#     dot_x = np.zeros(12)

#     # 3. Calculate all 12 derivatives (dot_x)
#     #    - dot_pos = vel
#     #    - dot_angles = W @ rates
#     #    - dot_vel = (1/m) * ([0,0,-mg] + R @ [0,0,-T])
#     #    - dot_rates = I_inv @ (torques - rates.cross(I @ rates))

#     # 4. Return dot_x (as a 1D array)
#     return dot_x

# from scipy.integrate import solve_ivp
# # --- Physical Constants ---
# g = 9.81        # gravitational acceleration (m/s^2)
# m = 1.0         # mass of quadcopter (kg)
# L = 0.25        # arm length (m)
# k_d = 0.1       # drag coefficient
# I = np.eye(3)   # moment of inertia matrix (3x3 identity as placeholder)
# M = np.eye(4)   # motor mixing matrix (placeholder for now)

# # --- Simulation Setup ---
# dt = 0.01  # Control loop time step (e.g., 100 Hz)
# total_time = 10.0
# num_steps = int(total_time / dt)

# # Initial state vector
# x_initial = np.zeros(12) 

# # Store results
# t_values = np.zeros(num_steps)
# x_values = np.zeros((12, num_steps))
# t_values[0] = 0.0
# x_values[:, 0] = x_initial

# # --- Initialize Controllers ---
# pid_z = PIDController(Kp=1.0, Ki=0.5, Kd=0.1)
# pid_roll = PIDController(Kp=1.2, Ki=0.4, Kd=0.1)
# pid_pitch = PIDController(Kp=1.2, Ki=0.4, Kd=0.1)
# pid_yaw = PIDController(Kp=0.8, Ki=0.3, Kd=0.05)

# # --- Main Loop ---
# for i in range(1, num_steps):
#     t_start = t_values[i-1]
#     t_end = t_start + dt
#     x_current = x_values[:, i-1]

#     # 1. --- CONTROL STEP ---
#     # Get desired setpoints (e.g., hover at z=1, 0 angles)
#     # Get desired setpoints (e.g., hover at z=1, 0 angles)
#     z_desired = 1.0
#     roll_desired = 0.0
#     pitch_desired = 0.0
#     yaw_desired = 0.0

#     # Calculate errors
#     error_z = z_desired - x_current[2]    # z is x[2]
#     error_roll = roll_desired - x_current[3]  # phi is x[3]
#     error_pitch = pitch_desired - x_current[4]  # theta is x[4]
#     error_yaw = yaw_desired - x_current[5]      # psi is x[5]


#     # Get "virtual" control outputs from PIDs
#     u_z = pid_z.update(error_z, dt)
#     u_phi = pid_roll.update(error_roll, dt)
#     u_theta = pid_pitch.update(error_pitch, dt)
#     u_psi = pid_yaw.update(error_yaw, dt)

#     # Add gravity compensation feed-forward to thrust
#     T = u_z + (m * g) 

#     # 2. --- CONTROL ALLOCATION ---
#     # Combine into virtual command vector
#     u_virtual = np.array([T, u_phi, u_theta, u_psi])

#     # Calculate actual motor thrusts
#     T_motors = np.linalg.inv(M) @ u_virtual

#     # (Optional) Saturate motors (e.g., T_i = max(0, min(T_i, T_MAX)))
#     T1, T2, T3, T4 = T_motors

#     # 3. --- DYNAMICS STEP ---
#     # Pass motor thrusts as *constant arguments* for this step
#     sol = solve_ivp(quadcopter_dynamics, [t_start, t_end], x_current,
#                     method='RK45', 
#                     args=(T1, T2, T3, T4, m, I, L, k_d))

#     # 4. --- STORE RESULTS ---
#     t_values[i] = sol.t[-1]
#     x_values[:, i] = sol.y[:, -1]