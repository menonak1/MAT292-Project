import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

class PIDController:
    """A simple PID controller class."""
    def __init__(self, Kp, Ki, Kd):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_error = 0.0
        self.previous_error = 0.0

    def update(self, error, dt):
        """Calculate the PID control output."""
        # Proportional term
        p_term = self.Kp * error

        # Integral term
        self.integral_error += error * dt
        i_term = self.Ki * self.integral_error

        # Derivative term
        derivative_error = (error - self.previous_error) / dt
        d_term = self.Kd * derivative_error

        # Update previous error for next iteration
        self.previous_error = error

        return p_term + i_term + d_term
    
    def reset(self):
        """Resets the integral and previous error."""
        self.integral_error = 0.0
        self.previous_error = 0.0

def quadcopter_dynamics(t, x, m, g, I, L, k_m, T1, T2, T3, T4):
    """
    Defines the 12-state quadcopter dynamics function.
    
    Inputs:
        t: time (not used, but required by solve_ivp)
        x: 12-element state vector
           [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
        m, g, I, L, k_m: Physical constants (mass, gravity, inertia, arm length, torque coeff)
        T1, T2, T3, T4: Motor thrusts (control inputs)
    
    Output:
        dot_x: 12-element vector of state derivatives
    """
    
    # --- 1. Unpack State Vector ---
    pos = x[0:3]       # [x, y, z]
    angles = x[3:6]    # [phi, theta, psi]
    vel = x[6:9]       # [x_dot, y_dot, z_dot]
    rates = x[9:12]    # [p, q, r]
    
    phi, theta, psi = angles
    p, q, r = rates
    
    # --- 2. Calculate Forces and Torques ---
    # Total thrust in Body frame
    T_total = T1 + T2 + T3 + T4
    F_thrust_B = np.array([0, 0, T_total])
    
    # Torques in Body frame (based on '+' configuration)
    tau_phi = L * (T2 - T3)
    tau_theta = L * (T1 - T4)
    tau_psi = k_m * (T1 - T2 + T3 - T4)
    tau_B = np.array([tau_phi, tau_theta, tau_psi])
    
    # --- 3. Precompute Trig Functions ---
    c_phi = np.cos(phi)
    s_phi = np.sin(phi)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    
    # --- 4. Calculate State Derivatives ---
    dot_x = np.zeros(12)
    
    # $\dot{p} = v$ (Derivative of position is velocity)
    dot_x[0:3] = vel
    
    # $\dot{\eta} = W \omega$ (Derivative of Euler angles)
    # Transformation matrix W
    W = np.array([
        [1, s_phi * np.tan(theta), c_phi * np.tan(theta)],
        [0, c_phi, -s_phi],
        [0, s_phi / c_theta, c_phi / c_theta]
    ])
    dot_x[3:6] = W @ rates
    
    # $\dot{v} = \ddot{p}$ (Derivative of velocity is acceleration)
    # Rotation matrix R (Body to Inertial)
    R = np.array([
        [c_psi * c_theta, c_psi * s_theta * s_phi - s_psi * c_phi, c_psi * s_theta * c_phi + s_psi * s_phi],
        [s_psi * c_theta, s_psi * s_theta * s_phi + c_psi * c_phi, s_psi * s_theta * c_phi - c_psi * s_phi],
        [-s_theta, c_theta * s_phi, c_theta * c_phi]
    ])
    
    # Gravity vector in Inertial frame
    F_grav_I = np.array([0, 0, -m * g])
    
    # Thrust in Inertial frame
    F_thrust_I = R @ F_thrust_B
    
    # Net force in Inertial frame
    F_net_I = F_grav_I + F_thrust_I
    
    # Translational acceleration
    dot_x[6:9] = F_net_I / m
    
    # $\dot{\omega} = \ddot{\eta}$ (Derivative of angular rates)
    # Newton-Euler equations
    I_inv = np.linalg.inv(I)
    # Gyroscopic terms (omega_B x (I @ omega_B))
    gyro_terms = np.cross(rates, I @ rates)
    
    dot_x[9:12] = I_inv @ (tau_B - gyro_terms)
    
    return dot_x

# --- Main Simulation ---
if __name__ == "__main__":

    # --- Physical Constants ---
    g = 9.81      # Gravitational acceleration (m/s^2)
    m = 0.5       # Mass of quadcopter (kg)
    L = 0.225     # Arm length (m)
    k_m = 0.01    # Yaw torque coefficient (N*m / N)
    # Inertia tensor (assuming a symmetrical frame)
    I_xx = 0.005
    I_yy = 0.005
    I_zz = 0.01
    I = np.diag([I_xx, I_yy, I_zz])
    
    # --- Simulation Setup ---
    dt = 0.01             # Control loop time step (s) -> 100 Hz
    total_time = 10.0     # Total simulation time (s)
    num_steps = int(total_time / dt)
    
    # Initial state vector [x, y, z, phi, theta, psi, x_dot, y_dot, z_dot, p, q, r]
    # Start on the ground, slightly tilted
    x_initial = np.zeros(12)
    x_initial[3] = np.deg2rad(1.0) # 1 degree roll
    x_initial[4] = np.deg2rad(-1.0) # -1 degree pitch

    # Store results
    t_values = np.zeros(num_steps)
    x_values = np.zeros((12, num_steps))
    t_values[0] = 0.0
    x_values[:, 0] = x_initial
    
    # Store motor thrusts for plotting
    motor_thrusts = np.zeros((4, num_steps))
    
    # --- Initialize Controllers ---
    # Tuned gains (These are placeholders, you must tune them!)
    # (Kp, Ki, Kd)
    pid_z = PIDController(0, 0, 0)
    pid_roll = PIDController(1.0, 0, 0)   # <-- This is the one you will test. Start the hunt from Kp=1.0
    pid_pitch = PIDController(10, 25, 0.5) # <-- GENTLE "HOLD" CONTROLLER (Prevents the crash)
    pid_yaw = PIDController(1.0, 20, 0.5)   # <-- GENTLE "HOLD" CONTROLLER (Prevents drift)
    
    # --- Control Allocation Matrix (Inverse) ---
    # T_m = M_inv @ u_v
    M_inv = np.array([
        [0.25, 0, 0.5/L, 0.25/k_m],
        [0.25, 0.5/L, 0, -0.25/k_m],
        [0.25, -0.5/L, 0, 0.25/k_m],
        [0.25, 0, -0.5/L, -0.25/k_m]
    ])
    
    # Max/Min motor thrust (for saturation)
    T_max = (m * g) / 2.0 # Max thrust per motor (e.g., 2x gravity)
    T_min = 0.0
    
    print("Starting simulation...")
    
    # --- Main Loop ---
    for i in range(1, num_steps):
        t_start = t_values[i-1]
        t_end = t_start + dt
        x_current = x_values[:, i-1]
        
        # --- 1. Define Setpoints (Desired State) ---
        # Hover at z=1.0m, no rotation
        z_desired = 1.0
        roll_desired = 0.0
        pitch_desired = 0.0
        yaw_desired = 0.0
        
        # Simple step disturbance for attitude
        if 2.0 < t_start < 6.0:
            roll_desired = np.deg2rad(10.0)
        
        # --- 2. Calculate Errors ---
        # Note: We control z-position, but attitude angles.
        # For a full controller, you'd control x,y position which would
        # generate desired roll/pitch, but for this project this is simpler.
        
        error_z = z_desired - x_current[2]       # z
        error_roll = roll_desired - x_current[3]   # phi
        error_pitch = pitch_desired - x_current[4] # theta
        error_yaw = yaw_desired - x_current[5]   # psi
        
        # PID controllers also need to dampen rates (D-term)
        # A better way is a "PD" controller on angle and a "P" on rate.
        # For simplicity, we'll use the PID as-is, but use angle *error*
        # and also pass in the *negative of the current rate* to the D-term
        # This is a common trick called "derivative on measurement"
        #
        # Here we'll stick to the textbook PID:
        # We need to compute the derivative of the error.
        # de/dt = (error - prev_error) / dt
        # For angles: d(phi_d - phi)/dt = -d(phi)/dt = -p (if phi_d is constant)
        # Let's modify the PID update to accept current_rate for the D-term
        
        # A standard PID's D-term is Kd * (error - prev_error) / dt
        # A PID with "Derivative on Measurement" is:
        # u(t) = Kp*e(t) + Ki*integral(e(t)) - Kd * (y(t) - y(t-1))/dt
        # For angles, this is approx: -Kd * (rate)
        
        # Let's just use the simple PID from the class for now.
        u_z = pid_z.update(error_z, dt)
        u_phi = pid_roll.update(error_roll, dt)
        u_theta = pid_pitch.update(error_pitch, dt)
        u_psi = pid_yaw.update(error_yaw, dt)

        # --- 3. Control Allocation ---
        
        # Add gravity compensation (feed-forward)
        # We must divide by (c_phi * c_theta) to account for tilt
        c_phi, c_theta = np.cos(x_current[3]), np.cos(x_current[4])
        T_hover = (m * g) / (c_phi * c_theta + 1e-6) # Add epsilon to avoid division by zero
        
        # Total thrust = hover thrust + altitude correction
        T = T_hover + u_z
        
        # Combine into virtual command vector [T, tau_phi, tau_theta, tau_psi]
        u_virtual = np.array([T, u_phi, u_theta, u_psi])
        
        # Calculate actual motor thrusts
        T_motors = M_inv @ u_virtual
        
        # Saturate motors
        T_motors_sat = np.clip(T_motors, T_min, T_max)
        T1, T2, T3, T4 = T_motors_sat
        motor_thrusts[:, i] = T_motors_sat
        
        # --- 4. Dynamics Step ---
        # Pass motor thrusts and constants as *constant arguments* for this step
        sol = solve_ivp(
            quadcopter_dynamics, 
            [t_start, t_end], 
            x_current,
            method='RK45',
            t_eval=[t_end], # Only get the final point
            args=(m, g, I, L, k_m, T1, T2, T3, T4)
        )
        
        # --- 5. Store Results ---
        t_values[i] = sol.t[-1]
        x_values[:, i] = sol.y[:, -1]
        
    print("Simulation complete.")

    # --- 6. Plot Results ---
    
    # Plot Altitude (z) and Position (x, y)
    plt.figure(figsize=(12, 6))
    plt.subplot(1, 2, 1)
    plt.plot(t_values, x_values[2, :], label='z (Altitude)')
    plt.plot(t_values, x_values[0, :], label='x', linestyle='--')
    plt.plot(t_values, x_values[1, :], label='y', linestyle='--')
    plt.axhline(y=1.0, color='r', linestyle=':', label='z desired')
    plt.title('Position')
    plt.xlabel('Time (s)')
    plt.ylabel('Position (m)')
    plt.legend()
    plt.grid(True)
    
    # Plot Attitude (Roll, Pitch, Yaw)
    plt.subplot(1, 2, 2)
    plt.plot(t_values, np.rad2deg(x_values[3, :]), label='$\phi$ (Roll)')
    plt.plot(t_values, np.rad2deg(x_values[4, :]), label='$\\theta$ (Pitch)')
    plt.plot(t_values, np.rad2deg(x_values[5, :]), label='$\psi$ (Yaw)')
    plt.axhline(y=10.0, xmin=0.2, xmax=0.6, color='r', linestyle=':', label='Roll disturbance')
    plt.title('Attitude')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()

    # Plot Motor Thrusts
    plt.figure(figsize=(10, 5))
    plt.plot(t_values, motor_thrusts[0, :], label='Motor 1 (Rear)')
    plt.plot(t_values, motor_thrusts[1, :], label='Motor 2 (Right)')
    plt.plot(t_values, motor_thrusts[2, :], label='Motor 3 (Left)')
    plt.plot(t_values, motor_thrusts[3, :], label='Motor 4 (Front)')
    plt.title('Motor Thrusts')
    plt.xlabel('Time (s)')
    plt.ylabel('Thrust (N)')
    plt.legend()
    plt.grid(True)

    plt.show()