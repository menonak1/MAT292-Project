import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

class PIDController:
    """A simple PID controller class with optional anti-windup and
    derivative-on-measurement support.
    """
    def __init__(self, Kp, Ki, Kd, integral_limit=None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_error = 0.0
        self.previous_error = 0.0
        self.integral_limit = integral_limit  # scalar limit for |integral_error|

    def update(self, error, dt, derivative=None):
        """Calculate the PID control output."""
        # Proportional term
        p_term = self.Kp * error

        # Integral term (with optional anti-windup clamp)
        self.integral_error += error * dt
        if self.integral_limit is not None and self.Ki != 0:
            lim = abs(self.integral_limit)
            self.integral_error = max(-lim, min(lim, self.integral_error))
        i_term = self.Ki * self.integral_error

        # Derivative term: Use provided derivative (e.g., negative body rate)
        if derivative is None:
            # Fallback to finite difference (less accurate for fast dynamics)
            derivative_error = (error - self.previous_error) / dt
        else:
            derivative_error = derivative
        d_term = self.Kd * derivative_error

        # Update previous error for next iteration
        self.previous_error = error

        return p_term + i_term + d_term
    
    def reset(self):
        """Resets the integral and previous error."""
        self.integral_error = 0.0
        self.previous_error = 0.0

def quadcopter_dynamics(t, x, m, g, I, L, k_m, c_d, T1, T2, T3, T4):
    """
    Defines the 12-state quadcopter dynamics function based on Newton-Euler equations.
    """
    
    # --- 1. Unpack State Vector ---
    angles = x[3:6]    # [phi, theta, psi]
    vel = x[6:9]       # [x_dot, y_dot, z_dot]
    rates = x[9:12]    # [p, q, r]
    
    phi, theta, psi = angles
    
    # --- 2. Calculate Forces and Torques (CRITICAL FIXES HERE) ---
    T_total = T1 + T2 + T3 + T4
    F_thrust_B = np.array([0, 0, T_total])
    
    # Torques in Body frame (updated '+' configuration with invertible yaw pattern)
    # Roll torque (motor 2 - motor 3)
    tau_phi = L * (T2 - T3)
    # Pitch torque (motor 1 - motor 4)
    tau_theta = L * (T1 - T4)
    # Yaw torque (motor spin pattern chosen so inverse exists): (T1 - T2 - T3 + T4)
    tau_psi = k_m * (T1 - T2 - T3 + T4)
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
    
    # $\dot{\eta} = W \omega$ (Derivative of Euler angles from Body rates)
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
    
    F_grav_I = np.array([0, 0, -m * g])
    F_thrust_I = R @ F_thrust_B
    # Linear aerodynamic drag (simple model): F_drag_I = -c_d * v_inertial
    F_drag_I = -c_d * vel
    F_net_I = F_grav_I + F_thrust_I + F_drag_I
    
    # Translational acceleration
    dot_x[6:9] = F_net_I / m
    
    # $\dot{\omega} = \ddot{\eta}$ (Derivative of angular rates)
    I_inv = np.linalg.inv(I)
    gyro_terms = np.cross(rates, I @ rates) # Gyroscopic terms $\omega \times I \omega$
    
    dot_x[9:12] = I_inv @ (tau_B - gyro_terms)
    
    return dot_x

# --- Main Simulation ---
if __name__ == "__main__":

    # --- Physical Constants ---
    g = 9.81      # Gravitational acceleration (m/s^2)
    m = 0.5       # Mass of quadcopter (kg)
    L = 0.225     # Arm length (m)
    k_m = 0.01    # Yaw torque coefficient (N*m / N)
    I_xx = 0.005
    I_yy = 0.005
    I_zz = 0.01
    I = np.diag([I_xx, I_yy, I_zz])
    
    # --- Simulation Setup ---
    dt = 0.01             # Control loop time step (s) -> 100 Hz
    total_time = 10.0     # Total simulation time (s)
    num_steps = int(total_time / dt)

    x_initial = np.zeros(12)
    x_initial[3] = np.deg2rad(1.0) # Initial 1 degree roll (disturbance)
    x_initial[4] = np.deg2rad(-1.0) # Initial -1 degree pitch (disturbance)

    t_values = np.zeros(num_steps)
    x_values = np.zeros((12, num_steps))
    t_values[0] = 0.0
    x_values[:, 0] = x_initial
    motor_thrusts = np.zeros((4, num_steps))
    
    # --- Initialize Controllers ---
    # We are resuming the Kp-only hunt for Roll, with Kp/Ki gentle holds on Pitch/Yaw
    # Integral limit prevents windup when Ki is used (e.g., in z, or if a hold loop is left running)
    
    # Kp/Ki/Kd/Integral_Limit
    
    # OFF - Z will crash to ground, but we are testing attitude stability only
    pid_z = PIDController(Kp=0.0, Ki=0.0, Kd=0.0, integral_limit=2.0)      
    pid_roll = PIDController(Kp=8.0, Ki=5.0, Kd=3.0, integral_limit=0.5)   
    pid_pitch = PIDController(Kp=1.0, Ki=0.1, Kd=0.0, integral_limit=0.5)  
    pid_yaw = PIDController(Kp=1.0, Ki=0.1, Kd=0.0, integral_limit=0.5)    

    # --- Control Allocation Matrix (M_inv) ---
    # INVERSE of the standard quadcopter mixing matrix (T, tau_phi, tau_theta, tau_psi -> T1, T2, T3, T4)
    # Corrected to reflect the T1-T2+T3-T4 yaw formula above.
    M_inv = np.array([
        [0.25, 0.0,  0.5 / L,  0.25 / k_m],    # T1
        [0.25, 0.5 / L, 0.0,  -0.25/ k_m],    # T2
        [0.25, -0.5 / L, 0.0, -0.25 / k_m],    # T3 (yaw sign flipped)
        [0.25, 0.0, -0.5 / L,  0.25 / k_m]     # T4 (yaw sign flipped)
    ])
    
    T_max = (m * g) * 2.0 # Max thrust (2x gravity)
    T_min = 0.0
    
    print("Starting simulation...")
    
    # --- Main Loop ---
    for i in range(1, num_steps):
        t_start = t_values[i-1]
        t_end = t_start + dt
        x_current = x_values[:, i-1]
        
        # --- 1. Define Setpoints (Desired State) ---
        z_desired = 1.0
        roll_desired = 0.0
        pitch_desired = 0.0
        yaw_desired = 0.0
        
        # Step disturbance (roll command)
        if 2.0 < t_start < 8.0:
            roll_desired = np.deg2rad(10.0)
        
        # --- 2. Calculate Errors & Control Output ---
        error_z = z_desired - x_current[2]       
        error_roll = roll_desired - x_current[3]   
        error_pitch = pitch_desired - x_current[4] 
        error_yaw = yaw_desired - x_current[5]   
        
        # Get body rates for Derivative-on-Measurement
        z_dot = x_current[8]
        p, q, r = x_current[9], x_current[10], x_current[11]

        # Use -rate as the derivative input (as rate is the derivative of angle in body frame)
        u_z = pid_z.update(error_z, dt, derivative=-z_dot)
        u_phi = pid_roll.update(error_roll, dt, derivative=-p)
        u_theta = pid_pitch.update(error_pitch, dt, derivative=-q)
        u_psi = pid_yaw.update(error_yaw, dt, derivative=-r)

        # --- 3. Control Allocation ---
        
        # Gravity compensation feed-forward: T_hover = mg / (cos(phi)cos(theta))
        c_phi, c_theta = np.cos(x_current[3]), np.cos(x_current[4])
        T_hover = (m * g) / (c_phi * c_theta + 1e-6) 
        
        # Total thrust = hover thrust + altitude correction
        T = T_hover + u_z
        
        u_virtual = np.array([T, u_phi, u_theta, u_psi])

        # Calculate motor thrusts
        T_motors = M_inv @ u_virtual

        # Saturate motors
        T_motors_sat = np.clip(T_motors, T_min, T_max)
        T1, T2, T3, T4 = T_motors_sat
        motor_thrusts[:, i] = T_motors_sat
        
        # --- 4. Dynamics Step ---
        sol = solve_ivp(
            quadcopter_dynamics,
            [t_start, t_end],
            x_current,
            method='RK45',
            t_eval=[t_end],
            args=(m, g, I, L, k_m, 0.1, T1, T2, T3, T4)  # c_d=0.1 drag coefficient
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
    plt.plot(t_values, np.rad2deg(x_values[0, :]), label='x (drift)', linestyle='--')
    plt.plot(t_values, np.rad2deg(x_values[1, :]), label='y (drift)', linestyle='--')
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
    plt.axhline(y=10.0, xmin=0.2, xmax=0.8, color='r', linestyle=':', label='Roll disturbance')
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