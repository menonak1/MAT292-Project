pid_z = PIDController(0, 0, 0)
    pid_roll = PIDController(10 , 25, 0.1)
    pid_pitch = PIDController(0, 0, 0) # Same as roll
    pid_yaw = PIDController(10, 1.0, 0.1)